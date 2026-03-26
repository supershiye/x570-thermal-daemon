#!/usr/bin/env python3
"""x570-thermal-daemon

A lightweight root daemon that maps a Windows FanControl curve to Linux hwmon PWM
outputs using a virtual water temperature derived from a time-averaged max(CPU, GPU).
"""

import json
import os
import signal
import time
from collections import deque
from typing import Deque, Dict, List, Optional


HARDWARE_MAP: Dict[str, Dict[str, object]] = {
    "sensors": {
        "CPU_GPU_MAX": [
            "/sys/class/hwmon/hwmon1/temp1_input",  # CPU temp input (adjust as needed)
            "/sys/class/hwmon/hwmon2/temp1_input",  # GPU temp input (adjust as needed)
        ]
    },
    "pwms": {
        "Bottom Fan": "/sys/class/hwmon/hwmon3/pwm2",
        "Side Fan": "/sys/class/hwmon/hwmon3/pwm3",
    },
}


class ThermalDaemon:
    """Thermal control daemon with liquid-loop inertia simulation."""

    def __init__(
        self,
        config_path: str,
        curve_name: str = "Auto Water Cooling",
        window_seconds: int = 180,
        poll_interval_seconds: float = 1.0,
    ) -> None:
        if poll_interval_seconds <= 0:
            raise ValueError("poll_interval_seconds must be > 0")
        if window_seconds <= 0:
            raise ValueError("window_seconds must be > 0")

        self.config_path: str = config_path
        self.curve_name: str = curve_name
        self.window_seconds: int = window_seconds
        self.poll_interval_seconds: float = poll_interval_seconds

        self.idle_temp_c: float = 0.0
        self.load_temp_c: float = 0.0
        self.min_fan_pct: float = 0.0
        self.max_fan_pct: float = 0.0

        self.sensor_paths: List[str] = self._load_sensor_paths()
        self.pwm_paths: List[str] = self._load_pwm_paths()

        window_samples: int = max(1, int(round(window_seconds / poll_interval_seconds)))
        self.temperature_window: Deque[float] = deque(maxlen=window_samples)

        self._running: bool = True
        self._last_valid_max_temp_c: Optional[float] = None

        self._load_curve_from_config()

    def _load_sensor_paths(self) -> List[str]:
        sensors = HARDWARE_MAP.get("sensors", {})
        sensor_paths = sensors.get("CPU_GPU_MAX") if isinstance(sensors, dict) else None
        if not isinstance(sensor_paths, list) or not sensor_paths:
            raise ValueError("HARDWARE_MAP['sensors']['CPU_GPU_MAX'] must be a non-empty list")
        return [str(path) for path in sensor_paths]

    def _load_pwm_paths(self) -> List[str]:
        pwms = HARDWARE_MAP.get("pwms", {})
        if not isinstance(pwms, dict) or not pwms:
            raise ValueError("HARDWARE_MAP['pwms'] must be a non-empty dict")
        return [str(path) for path in pwms.values()]

    def _load_curve_from_config(self) -> None:
        """Extract only the curve nodes required for linear interpolation."""
        try:
            with open(self.config_path, "r", encoding="utf-8") as file_handle:
                config = json.load(file_handle)
        except OSError as exc:
            raise RuntimeError(f"Failed to open config '{self.config_path}': {exc}") from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON in '{self.config_path}': {exc}") from exc

        fan_control = config.get("FanControl")
        if not isinstance(fan_control, dict):
            raise RuntimeError("Missing or invalid 'FanControl' object in config")

        fan_curves = fan_control.get("FanCurves")
        if not isinstance(fan_curves, list):
            raise RuntimeError("Missing or invalid 'FanCurves' array in config")

        curve: Optional[dict] = None
        for candidate in fan_curves:
            if isinstance(candidate, dict) and candidate.get("Name") == self.curve_name:
                curve = candidate
                break

        if curve is None:
            raise RuntimeError(f"Curve '{self.curve_name}' not found in FanCurves")

        try:
            self.idle_temp_c = float(curve["IdleTemperature"])
            self.load_temp_c = float(curve["LoadTemperature"])
            self.min_fan_pct = float(curve["MinFanSpeed"])
            self.max_fan_pct = float(curve["MaxFanSpeed"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"Invalid curve fields in '{self.curve_name}': {exc}") from exc

        if self.load_temp_c <= self.idle_temp_c:
            raise RuntimeError("Curve must satisfy LoadTemperature > IdleTemperature")
        if not (0.0 <= self.min_fan_pct <= 100.0 and 0.0 <= self.max_fan_pct <= 100.0):
            raise RuntimeError("Curve fan percentages must be in [0, 100]")

    def _read_temperature_c(self, path: str) -> float:
        """Read hwmon temperature in millidegrees C and convert to degrees C."""
        with open(path, "r", encoding="utf-8") as file_handle:
            raw = file_handle.read().strip()
        return float(raw) / 1000.0

    def _read_current_max_temp_c(self) -> float:
        """Read all mapped sensors and return their max temperature.

        If any read fails, use a safe fallback to avoid under-cooling.
        """
        values: List[float] = []
        for sensor_path in self.sensor_paths:
            try:
                values.append(self._read_temperature_c(sensor_path))
            except (OSError, ValueError) as exc:
                print(f"WARN: Failed reading sensor '{sensor_path}': {exc}")

        if values:
            current_max = max(values)
            self._last_valid_max_temp_c = current_max
            return current_max

        # Safe fallback path when all sensors are unavailable.
        if self._last_valid_max_temp_c is not None:
            print("WARN: Sensor read failed; reusing last valid max temperature")
            return self._last_valid_max_temp_c

        print("WARN: Sensor read failed; falling back to load temperature for safety")
        return self.load_temp_c

    def _compute_virtual_water_temp_c(self, current_max_temp_c: float) -> float:
        self.temperature_window.append(current_max_temp_c)
        return sum(self.temperature_window) / float(len(self.temperature_window))

    def _interpolate_fan_percent(self, temperature_c: float) -> float:
        """Piecewise-linear mapping from temperature to fan percent."""
        if temperature_c <= self.idle_temp_c:
            return self.min_fan_pct
        if temperature_c >= self.load_temp_c:
            return self.max_fan_pct

        slope = (self.max_fan_pct - self.min_fan_pct) / (self.load_temp_c - self.idle_temp_c)
        return self.min_fan_pct + slope * (temperature_c - self.idle_temp_c)

    def _fan_percent_to_pwm(self, fan_percent: float) -> int:
        clamped = max(0.0, min(100.0, fan_percent))
        return int(round((clamped / 100.0) * 255.0))

    def _write_pwm_value(self, pwm_path: str, pwm_value: int) -> bool:
        try:
            with open(pwm_path, "w", encoding="utf-8") as file_handle:
                file_handle.write(f"{pwm_value}\n")
            return True
        except OSError as exc:
            print(f"ERROR: Failed writing PWM '{pwm_path}': {exc}")
            return False

    def _write_all_pwms(self, pwm_value: int) -> None:
        for pwm_path in self.pwm_paths:
            self._write_pwm_value(pwm_path, pwm_value)

    def _handle_stop_signal(self, signum: int, _frame: object) -> None:
        print(f"INFO: Received signal {signum}; requesting shutdown")
        self._running = False

    def install_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_stop_signal)
        signal.signal(signal.SIGTERM, self._handle_stop_signal)

    def run(self) -> None:
        print("INFO: x570-thermal-daemon starting")
        print(f"INFO: Config path: {self.config_path}")
        print(
            "INFO: Curve "
            f"{self.curve_name} => idle={self.idle_temp_c:.1f}C,min={self.min_fan_pct:.1f}% "
            f"load={self.load_temp_c:.1f}C,max={self.max_fan_pct:.1f}%"
        )
        print(f"INFO: Poll interval={self.poll_interval_seconds:.1f}s window={self.window_seconds}s")

        while self._running:
            cycle_start = time.time()

            current_max_temp_c = self._read_current_max_temp_c()
            virtual_water_temp_c = self._compute_virtual_water_temp_c(current_max_temp_c)
            target_pct = self._interpolate_fan_percent(virtual_water_temp_c)
            target_pwm = self._fan_percent_to_pwm(target_pct)

            success_count = 0
            for pwm_path in self.pwm_paths:
                if self._write_pwm_value(pwm_path, target_pwm):
                    success_count += 1

            print(
                "INFO: "
                f"max={current_max_temp_c:.2f}C "
                f"virtual_water={virtual_water_temp_c:.2f}C "
                f"target={target_pct:.1f}% ({target_pwm}/255) "
                f"writes={success_count}/{len(self.pwm_paths)}"
            )

            elapsed = time.time() - cycle_start
            sleep_time = self.poll_interval_seconds - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.shutdown()

    def shutdown(self) -> None:
        print("INFO: Writing failsafe PWM=255 to all managed channels")
        self._write_all_pwms(255)
        print("INFO: x570-thermal-daemon stopped")


def main() -> int:
    config_path = os.environ.get("X570_THERMAL_CONFIG", "userConfig.json")
    curve_name = os.environ.get("X570_THERMAL_CURVE", "Auto Water Cooling")

    window_seconds_env = os.environ.get("X570_THERMAL_WINDOW_SECONDS", "180")
    poll_interval_env = os.environ.get("X570_THERMAL_POLL_INTERVAL", "1.0")

    try:
        window_seconds = int(window_seconds_env)
        poll_interval_seconds = float(poll_interval_env)
    except ValueError as exc:
        print(f"ERROR: Invalid numeric environment override: {exc}")
        return 1

    try:
        daemon = ThermalDaemon(
            config_path=config_path,
            curve_name=curve_name,
            window_seconds=window_seconds,
            poll_interval_seconds=poll_interval_seconds,
        )
    except (ValueError, RuntimeError) as exc:
        print(f"ERROR: Failed initializing daemon: {exc}")
        return 1

    daemon.install_signal_handlers()

    try:
        daemon.run()
    except KeyboardInterrupt:
        # SIGINT should normally flip _running, but keep this as a hard fallback.
        daemon.shutdown()
    except Exception as exc:  # pylint: disable=broad-except
        print(f"ERROR: Unhandled runtime error: {exc}")
        daemon.shutdown()
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
