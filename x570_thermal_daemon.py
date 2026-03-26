#!/usr/bin/env python3
"""x570-thermal-daemon

Production-grade daemon that maps a Windows FanControl profile to Linux hwmon PWM
outputs while simulating liquid-loop thermal inertia through a rolling temperature
average.
"""

import json
import os
import signal
import subprocess
import time
from collections import deque
from typing import Deque, Dict, List, Optional


DEFAULT_HARDWARE_MAP_PATH = "hardware_map_msi_x570_unify.json"
NVIDIA_SMI_QUERY_CMD = [
    "nvidia-smi",
    "--query-gpu=temperature.gpu",
    "--format=csv,noheader",
]


class ThermalDaemon:  # pylint: disable=too-many-instance-attributes
    """Thermal control daemon for custom liquid cooling loops."""

    def __init__(  # pylint: disable=too-many-arguments
        self,
        config_path: str,
        hardware_map_path: str = DEFAULT_HARDWARE_MAP_PATH,
        curve_name: str = "Auto Water Cooling",
        window_seconds: int = 180,
        poll_interval_seconds: float = 1.0,
    ) -> None:
        if window_seconds <= 0:
            raise ValueError("window_seconds must be > 0")
        if poll_interval_seconds <= 0:
            raise ValueError("poll_interval_seconds must be > 0")

        self.config_path: str = config_path
        self.hardware_map_path: str = hardware_map_path
        self.curve_name: str = curve_name
        self.window_seconds: int = window_seconds
        self.poll_interval_seconds: float = poll_interval_seconds

        self.idle_temp_c: float = 0.0
        self.load_temp_c: float = 0.0
        self.min_fan_pct: float = 0.0
        self.max_fan_pct: float = 0.0

        self.cpu_temp_path: str = ""
        self.gpu_method: str = ""
        self.control_map: Dict[str, str] = {}

        self.target_control_ids: List[str] = []
        self.target_pwm_paths: List[str] = []
        self.all_pwm_paths: List[str] = []

        self._window_samples: int = max(1, int(round(window_seconds / poll_interval_seconds)))
        self.temperature_window: Deque[float] = deque()
        self._temperature_window_sum: float = 0.0

        self._running: bool = True
        self._shutdown_complete: bool = False
        self._last_valid_max_temp_c: Optional[float] = None
        self._permission_denied_paths: Dict[str, bool] = {}
        self._consecutive_full_write_failures: int = 0

        self._load_hardware_map()
        self._load_fancontrol_config()

    def _load_hardware_map(self) -> None:  # pylint: disable=too-many-branches
        """Load hardware I/O mapping from the provided JSON file."""
        try:
            with open(self.hardware_map_path, "r", encoding="utf-8") as file_handle:
                raw_map: object = json.load(file_handle)
        except OSError as exc:
            raise RuntimeError(
                f"Failed opening hardware map '{self.hardware_map_path}': {exc}"
            ) from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Invalid JSON in hardware map '{self.hardware_map_path}': {exc}"
            ) from exc

        if not isinstance(raw_map, dict):
            raise RuntimeError("Hardware map root must be a JSON object")

        temperature_sensors = raw_map.get("temperature_sensors")
        if not isinstance(temperature_sensors, dict):
            raise RuntimeError("Hardware map must contain object key 'temperature_sensors'")

        cpu_temp_path = temperature_sensors.get("CPU_TEMP_PATH")
        if not isinstance(cpu_temp_path, str) or not cpu_temp_path.strip():
            raise RuntimeError("'temperature_sensors.CPU_TEMP_PATH' must be a non-empty string")

        gpu_method = temperature_sensors.get("GPU_METHOD")
        if not isinstance(gpu_method, str) or not gpu_method.strip():
            raise RuntimeError("'temperature_sensors.GPU_METHOD' must be a non-empty string")

        if gpu_method != "nvidia_smi_query":
            raise RuntimeError(
                "Unsupported GPU method in hardware map. "
                "Expected 'nvidia_smi_query'."
            )

        controls = raw_map.get("controls")
        if not isinstance(controls, dict) or not controls:
            raise RuntimeError("Hardware map must contain non-empty object key 'controls'")

        normalized_controls: Dict[str, str] = {}
        for control_identifier, pwm_path in controls.items():
            if not isinstance(control_identifier, str) or not control_identifier.strip():
                raise RuntimeError("All 'controls' keys must be non-empty strings")
            if not isinstance(pwm_path, str) or not pwm_path.strip():
                raise RuntimeError("All 'controls' values must be non-empty strings")
            normalized_controls[control_identifier.strip()] = pwm_path.strip()

        self.cpu_temp_path = cpu_temp_path.strip()
        self.gpu_method = gpu_method.strip()
        self.control_map = normalized_controls
        self.all_pwm_paths = self._unique_preserve_order(list(self.control_map.values()))

    def _load_fancontrol_config(self) -> None:
        """Load curve settings and discover which controls use that curve."""
        try:
            with open(self.config_path, "r", encoding="utf-8") as file_handle:
                config = json.load(file_handle)
        except OSError as exc:
            raise RuntimeError(f"Failed opening config '{self.config_path}': {exc}") from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON in config '{self.config_path}': {exc}") from exc

        fan_control = config.get("FanControl")
        if not isinstance(fan_control, dict):
            raise RuntimeError("Config is missing object key 'FanControl'")

        self._extract_curve_points(fan_control)
        self._extract_target_controls(fan_control)

    def _extract_curve_points(self, fan_control: Dict[str, object]) -> None:
        """Extract linear curve points from FanCurves for the target curve name."""
        fan_curves = fan_control.get("FanCurves")
        if not isinstance(fan_curves, list):
            raise RuntimeError("Config is missing array key 'FanControl.FanCurves'")

        selected_curve: Optional[dict] = None
        for curve_candidate in fan_curves:
            if not isinstance(curve_candidate, dict):
                continue
            if curve_candidate.get("Name") == self.curve_name:
                selected_curve = curve_candidate
                break

        if selected_curve is None:
            raise RuntimeError(f"Fan curve '{self.curve_name}' not found")

        self.idle_temp_c = self._require_float_field(selected_curve, "IdleTemperature")
        self.load_temp_c = self._require_float_field(selected_curve, "LoadTemperature")
        self.min_fan_pct = self._require_float_field(selected_curve, "MinFanSpeed")
        self.max_fan_pct = self._require_float_field(selected_curve, "MaxFanSpeed")

        if self.load_temp_c <= self.idle_temp_c:
            raise RuntimeError("LoadTemperature must be greater than IdleTemperature")
        if not 0.0 <= self.min_fan_pct <= 100.0:
            raise RuntimeError("MinFanSpeed must be in [0, 100]")
        if not 0.0 <= self.max_fan_pct <= 100.0:
            raise RuntimeError("MaxFanSpeed must be in [0, 100]")

    def _require_float_field(self, data: Dict[str, object], key: str) -> float:
        """Read a required numeric field and return it as float."""
        if key not in data:
            raise RuntimeError(f"Missing required curve field '{key}'")

        raw_value = data[key]
        if isinstance(raw_value, (int, float)):
            return float(raw_value)

        if isinstance(raw_value, str):
            stripped = raw_value.strip()
            if not stripped:
                raise RuntimeError(f"Field '{key}' is empty")
            try:
                return float(stripped)
            except ValueError as exc:
                raise RuntimeError(f"Field '{key}' is not a valid number: {raw_value}") from exc

        raise RuntimeError(f"Field '{key}' has unsupported type: {type(raw_value).__name__}")

    def _extract_target_controls(self, fan_control: Dict[str, object]) -> None:
        """Select only controls assigned to the requested curve name."""
        controls = fan_control.get("Controls")
        if not isinstance(controls, list):
            raise RuntimeError("Config is missing array key 'FanControl.Controls'")

        selected_ids: List[str] = []
        for control in controls:
            if not isinstance(control, dict):
                continue

            if not bool(control.get("Enable", False)):
                continue

            selected_curve_obj = control.get("SelectedFanCurve")
            selected_curve_name = ""
            if isinstance(selected_curve_obj, dict):
                raw_curve_name = selected_curve_obj.get("Name")
                if isinstance(raw_curve_name, str):
                    selected_curve_name = raw_curve_name

            if selected_curve_name != self.curve_name:
                continue

            control_identifier = control.get("Identifier")
            if not isinstance(control_identifier, str) or not control_identifier.strip():
                continue
            control_identifier = control_identifier.strip()

            if control_identifier not in self.control_map:
                print(
                    "WARN: Control mapped to curve but missing from hardware map: "
                    f"{control_identifier}"
                )
                continue

            selected_ids.append(control_identifier)

        if not selected_ids:
            raise RuntimeError(
                f"No enabled controls mapped to fan curve '{self.curve_name}' were found"
            )

        self.target_control_ids = self._unique_preserve_order(selected_ids)
        self.target_pwm_paths = self._unique_preserve_order(
            [self.control_map[control_id] for control_id in self.target_control_ids]
        )

    def _unique_preserve_order(self, values: List[str]) -> List[str]:
        return list(dict.fromkeys(values))

    def _read_cpu_temp(self) -> float:
        """Read CPU temperature from hwmon path (millidegree C -> degree C)."""
        with open(self.cpu_temp_path, "r", encoding="utf-8") as file_handle:
            raw_value = file_handle.read().strip()
        return float(raw_value) / 1000.0

    def get_gpu_temp(self) -> float:
        """Read GPU temperature using nvidia-smi query command."""
        if self.gpu_method != "nvidia_smi_query":
            raise RuntimeError(f"Unsupported GPU method: {self.gpu_method}")

        try:
            output = subprocess.check_output(
                NVIDIA_SMI_QUERY_CMD,
                text=True,
                timeout=3.0,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("nvidia-smi was not found in PATH") from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f"nvidia-smi command failed: {exc}") from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"nvidia-smi command timed out: {exc}") from exc

        values: List[float] = []
        for line in output.splitlines():
            text_line = line.strip()
            if not text_line:
                continue
            # Defensive parsing in case output contains commas/spaces.
            first_field = text_line.split(",", maxsplit=1)[0].strip()
            values.append(float(first_field))

        if not values:
            raise RuntimeError("nvidia-smi returned no GPU temperature values")

        return max(values)

    def _read_current_max_temp(self) -> float:
        """Read CPU/GPU temperatures and return max(), with safe fallback."""
        current_max: Optional[float] = None

        try:
            current_max = self._read_cpu_temp()
        except (OSError, ValueError) as exc:
            print(f"WARN: Failed reading CPU temperature '{self.cpu_temp_path}': {exc}")

        try:
            gpu_temp = self.get_gpu_temp()
            if current_max is None or gpu_temp > current_max:
                current_max = gpu_temp
        except (RuntimeError, ValueError) as exc:
            print(f"WARN: Failed reading GPU temperature via nvidia-smi: {exc}")

        if current_max is not None:
            self._last_valid_max_temp_c = current_max
            return current_max

        if self._last_valid_max_temp_c is not None:
            print("WARN: Reusing last valid max temperature")
            return self._last_valid_max_temp_c

        print("WARN: All sensors unavailable; falling back to load temperature for safety")
        return self.load_temp_c

    def _virtual_water_temp(self, current_max_temp: float) -> float:
        """Maintain rolling average over the configured window duration."""
        if len(self.temperature_window) >= self._window_samples:
            self._temperature_window_sum -= self.temperature_window.popleft()

        self.temperature_window.append(current_max_temp)
        self._temperature_window_sum += current_max_temp
        return self._temperature_window_sum / float(len(self.temperature_window))

    def _interpolate_percent(self, temp_c: float) -> float:
        """Linearly map temperature to fan percent using idle/load endpoints."""
        if temp_c <= self.idle_temp_c:
            return self.min_fan_pct
        if temp_c >= self.load_temp_c:
            return self.max_fan_pct

        ratio = (temp_c - self.idle_temp_c) / (self.load_temp_c - self.idle_temp_c)
        return self.min_fan_pct + ratio * (self.max_fan_pct - self.min_fan_pct)

    def _percent_to_pwm(self, percent: float) -> int:
        clamped = max(0.0, min(100.0, percent))
        return int(round((clamped / 100.0) * 255.0))

    def _pwm_enable_path(self, pwm_path: str) -> str:
        base_dir = os.path.dirname(pwm_path)
        pwm_name = os.path.basename(pwm_path)
        return os.path.join(base_dir, f"{pwm_name}_enable")

    def _set_manual_mode(self, pwm_path: str) -> bool:
        """Attempt to force pwmX_enable=1 for direct user-space writes."""
        enable_path = self._pwm_enable_path(pwm_path)
        if not os.path.exists(enable_path):
            print(f"WARN: Missing PWM enable node '{enable_path}'")
            return False

        try:
            with open(enable_path, "w", encoding="utf-8") as file_handle:
                file_handle.write("1\n")
            return True
        except OSError as exc:
            print(f"WARN: Failed setting manual mode on '{enable_path}': {exc}")
            return False

    def _write_pwm(self, pwm_path: str, value: int) -> bool:
        try:
            with open(pwm_path, "w", encoding="utf-8") as file_handle:
                file_handle.write(f"{value}\n")
            return True
        except PermissionError as exc:
            if not self._permission_denied_paths.get(pwm_path, False):
                print(f"ERROR: Permission denied writing '{pwm_path}': {exc}")
                self._permission_denied_paths[pwm_path] = True
            return False
        except OSError as exc:
            print(f"ERROR: Failed writing '{pwm_path}': {exc}")
            return False

    def _preflight_checks(self) -> bool:
        """Validate root access, sensors, and selected PWM outputs."""
        if os.geteuid() != 0:
            print("ERROR: Must run as root to write sysfs PWM files")
            return False

        available_sources = 0
        try:
            _ = self._read_cpu_temp()
            available_sources += 1
        except (OSError, ValueError) as exc:
            print(f"WARN: CPU sensor preflight failed ('{self.cpu_temp_path}'): {exc}")

        try:
            _ = self.get_gpu_temp()
            available_sources += 1
        except (RuntimeError, ValueError) as exc:
            print(f"WARN: GPU sensor preflight failed: {exc}")

        if available_sources == 0:
            print("ERROR: No readable CPU/GPU sensors are available")
            return False

        preflight_ok = True
        for pwm_path in self.target_pwm_paths:
            if not os.path.exists(pwm_path):
                print(f"ERROR: Target PWM path not found: {pwm_path}")
                preflight_ok = False
                continue

            self._set_manual_mode(pwm_path)
            if not os.access(pwm_path, os.W_OK):
                print(f"ERROR: Target PWM path is not writable: {pwm_path}")
                preflight_ok = False

        return preflight_ok

    def _handle_stop_signal(self, signum: int, _frame: object) -> None:
        print(f"INFO: Received signal {signum}; preparing shutdown")
        self._running = False

    def install_signal_handlers(self) -> None:
        """Register SIGINT/SIGTERM handlers for graceful shutdown."""
        signal.signal(signal.SIGINT, self._handle_stop_signal)
        signal.signal(signal.SIGTERM, self._handle_stop_signal)

    def run(self) -> None:
        """Main daemon loop."""
        print("INFO: x570-thermal-daemon starting")
        print(f"INFO: Config: {self.config_path}")
        print(f"INFO: Hardware map: {self.hardware_map_path}")
        print(f"INFO: CPU temp path: {self.cpu_temp_path}")
        print(f"INFO: GPU method: {self.gpu_method}")
        print(f"INFO: Curve: {self.curve_name}")
        print(
            "INFO: Curve points "
            f"idle={self.idle_temp_c:.1f}C,min={self.min_fan_pct:.1f}% "
            f"load={self.load_temp_c:.1f}C,max={self.max_fan_pct:.1f}%"
        )
        print(
            f"INFO: Window={self.window_seconds}s Poll={self.poll_interval_seconds:.1f}s "
            f"Samples={self._window_samples}"
        )
        print(f"INFO: Controls on curve: {self.target_control_ids}")
        print(f"INFO: Target PWM paths: {self.target_pwm_paths}")

        if not self._preflight_checks():
            print("ERROR: Preflight checks failed; daemon not started")
            return

        target_pwm_count = len(self.target_pwm_paths)
        try:
            while self._running:
                cycle_start = time.time()

                current_max = self._read_current_max_temp()
                virtual_temp = self._virtual_water_temp(current_max)
                percent = self._interpolate_percent(virtual_temp)
                pwm_value = self._percent_to_pwm(percent)

                success_count = 0
                for pwm_path in self.target_pwm_paths:
                    if self._write_pwm(pwm_path, pwm_value):
                        success_count += 1

                print(
                    "INFO: "
                    f"max={current_max:.2f}C "
                    f"virtual={virtual_temp:.2f}C "
                    f"target={percent:.1f}% ({pwm_value}/255) "
                    f"writes={success_count}/{target_pwm_count}"
                )

                if success_count == 0:
                    self._consecutive_full_write_failures += 1
                else:
                    self._consecutive_full_write_failures = 0

                if self._consecutive_full_write_failures >= 10:
                    print("ERROR: 10 consecutive cycles with zero PWM writes; stopping")
                    self._running = False

                elapsed = time.time() - cycle_start
                sleep_time = self.poll_interval_seconds - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Failsafe: set all mapped PWM outputs to 100% on exit."""
        if self._shutdown_complete:
            return

        self._shutdown_complete = True
        print("INFO: Applying failsafe PWM=255 to all mapped controls")
        for pwm_path in self.all_pwm_paths:
            self._set_manual_mode(pwm_path)
            self._write_pwm(pwm_path, 255)
        print("INFO: x570-thermal-daemon stopped")


def main() -> int:
    """Program entrypoint."""
    config_path = os.environ.get("X570_THERMAL_CONFIG", "userConfig.json")
    hardware_map_path = os.environ.get("X570_THERMAL_MAP", DEFAULT_HARDWARE_MAP_PATH)
    curve_name = os.environ.get("X570_THERMAL_CURVE", "Auto Water Cooling")

    try:
        window_seconds = int(os.environ.get("X570_THERMAL_WINDOW_SECONDS", "180"))
        poll_interval_seconds = float(os.environ.get("X570_THERMAL_POLL_INTERVAL", "1.0"))
    except ValueError as exc:
        print(f"ERROR: Invalid numeric environment variable: {exc}")
        return 1

    try:
        daemon = ThermalDaemon(
            config_path=config_path,
            hardware_map_path=hardware_map_path,
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
    except Exception as exc:  # pylint: disable=broad-except
        print(f"ERROR: Unhandled runtime error: {exc}")
        daemon.shutdown()
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
