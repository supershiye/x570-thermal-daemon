#!/usr/bin/env python3
"""x570-thermal-daemon.

Production-grade, zero-dependency thermal daemon that bridges a Windows Fan Control
JSON profile to Linux hwmon PWM control on MSI X570/NCT6797D systems.
"""

import json
import os
import signal
import subprocess
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Set, Tuple


HARDWARE_MAP: Dict[str, Dict[str, object]] = {
    "temperature_sensors": {
        "CPU_TEMP_PATHS": [
            "/sys/class/hwmon/hwmon4/temp1_input",
            "/sys/class/hwmon/hwmon4/temp3_input",
            "/sys/class/hwmon/hwmon4/temp4_input",
        ],
        "GPU_METHOD": "nvidia_smi_query",
    },
    "controls": {
        "/lpc/nct6797d/control/0": "/sys/class/hwmon/hwmon3/pwm1",  # Pump Fan
        "/lpc/nct6797d/control/1": "/sys/class/hwmon/hwmon3/pwm2",  # Back Fan
        "/lpc/nct6797d/control/2": "/sys/class/hwmon/hwmon3/pwm3",  # Bottom Fan
        "/lpc/nct6797d/control/3": "/sys/class/hwmon/hwmon3/pwm4",  # Side Fan
        "/lpc/nct6797d/control/4": "/sys/class/hwmon/hwmon3/pwm5",  # Water Pump
        "/lpc/nct6797d/control/5": "/sys/class/hwmon/hwmon3/pwm6",  # Top Fan
        "/lpc/nct6797d/control/6": "/sys/class/hwmon/hwmon3/pwm7",  # Chipset Fan
    },
}

NVIDIA_SMI_QUERY_CMD: List[str] = [
    "nvidia-smi",
    "--query-gpu=temperature.gpu",
    "--format=csv,noheader",
]


class ThermalDaemon:
    """Evaluate all active curves concurrently and drive mapped PWM outputs."""

    def __init__(self, config_path: str, poll_interval_seconds: float = 1.0) -> None:
        if poll_interval_seconds <= 0.0:
            raise ValueError("poll_interval_seconds must be > 0")

        self.config_path: str = config_path
        self.poll_interval_seconds: float = poll_interval_seconds

        self.cpu_temp_paths: List[str] = []
        self.gpu_method: str = ""
        self.control_map: Dict[str, str] = {}
        self.nct_hwmon_dir: str = ""

        self.curves_by_name: Dict[str, Dict[str, object]] = {}
        self.custom_sensors_by_id: Dict[str, Dict[str, object]] = {}

        self.active_curve_to_control_ids: Dict[str, List[str]] = {}
        self.active_curve_to_pwm_paths: Dict[str, List[str]] = {}
        self.active_pwm_paths: List[str] = []

        self.time_windows: Dict[str, Deque[float]] = {}
        self.time_window_sums: Dict[str, float] = {}
        self.time_window_samples: Dict[str, int] = {}

        self.nvme_paths_by_index: Dict[int, Dict[int, str]] = {}

        self._cycle_cache: Dict[str, float] = {}
        self._running: bool = True
        self._shutdown_done: bool = False
        self._curve_error_state: Dict[str, str] = {}

        self._load_hardware_map()
        self._load_fancontrol_config()
        self._discover_nvme_temp_paths()
        self._validate_active_curve_graphs()

    def _load_hardware_map(self) -> None:
        """Validate and normalize in-code hardware map."""
        temp_block = HARDWARE_MAP.get("temperature_sensors")
        if not isinstance(temp_block, dict):
            raise RuntimeError("HARDWARE_MAP.temperature_sensors must be an object")

        cpu_paths_raw = temp_block.get("CPU_TEMP_PATHS")
        if not isinstance(cpu_paths_raw, list) or not cpu_paths_raw:
            raise RuntimeError("HARDWARE_MAP.temperature_sensors.CPU_TEMP_PATHS must be non-empty")

        normalized_cpu_paths: List[str] = []
        for item in cpu_paths_raw:
            if not isinstance(item, str) or not item.strip():
                raise RuntimeError("Every CPU_TEMP_PATHS entry must be a non-empty string")
            normalized_cpu_paths.append(item.strip())

        gpu_method_raw = temp_block.get("GPU_METHOD")
        if not isinstance(gpu_method_raw, str) or not gpu_method_raw.strip():
            raise RuntimeError("HARDWARE_MAP.temperature_sensors.GPU_METHOD must be a string")
        if gpu_method_raw.strip() != "nvidia_smi_query":
            raise RuntimeError("Only GPU_METHOD='nvidia_smi_query' is supported")

        controls_raw = HARDWARE_MAP.get("controls")
        if not isinstance(controls_raw, dict) or not controls_raw:
            raise RuntimeError("HARDWARE_MAP.controls must be a non-empty object")

        normalized_controls: Dict[str, str] = {}
        for control_identifier, pwm_path in controls_raw.items():
            if not isinstance(control_identifier, str) or not control_identifier.strip():
                raise RuntimeError("Every HARDWARE_MAP.controls key must be a non-empty string")
            if not isinstance(pwm_path, str) or not pwm_path.strip():
                raise RuntimeError("Every HARDWARE_MAP.controls value must be a non-empty string")
            normalized_controls[control_identifier.strip()] = pwm_path.strip()

        self.cpu_temp_paths = normalized_cpu_paths
        self.gpu_method = gpu_method_raw.strip()
        self.control_map = normalized_controls

        first_pwm_path = next(iter(self.control_map.values()))
        self.nct_hwmon_dir = os.path.dirname(first_pwm_path)

    def _load_fancontrol_config(self) -> None:
        """Parse all curves, custom sensors, and active control assignments."""
        try:
            with open(self.config_path, "r", encoding="utf-8") as file_handle:
                config = json.load(file_handle)
        except OSError as exc:
            raise RuntimeError(f"Failed reading config '{self.config_path}': {exc}") from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON in '{self.config_path}': {exc}") from exc

        fan_control = config.get("FanControl")
        if not isinstance(fan_control, dict):
            raise RuntimeError("Missing required object: FanControl")

        self._parse_all_curves(fan_control)
        self._parse_custom_sensors(fan_control)
        self._parse_active_controls(fan_control)
        self._initialize_time_windows()

    def _parse_all_curves(self, fan_control: Dict[str, object]) -> None:
        fan_curves = fan_control.get("FanCurves")
        if not isinstance(fan_curves, list):
            raise RuntimeError("Missing required array: FanControl.FanCurves")

        curves: Dict[str, Dict[str, object]] = {}
        for raw_curve in fan_curves:
            if not isinstance(raw_curve, dict):
                continue

            curve_name = self._require_str_field(raw_curve, "Name")
            idle_temp_c = self._require_float_field(raw_curve, "IdleTemperature")
            load_temp_c = self._require_float_field(raw_curve, "LoadTemperature")
            min_fan_pct = self._require_float_field(raw_curve, "MinFanSpeed")
            max_fan_pct = self._require_float_field(raw_curve, "MaxFanSpeed")

            selected_source = raw_curve.get("SelectedTempSource")
            if not isinstance(selected_source, dict):
                raise RuntimeError(f"Curve '{curve_name}' missing SelectedTempSource")
            source_identifier = self._require_str_field(selected_source, "Identifier")

            if load_temp_c <= idle_temp_c:
                raise RuntimeError(
                    f"Curve '{curve_name}' must satisfy LoadTemperature > IdleTemperature"
                )
            if not 0.0 <= min_fan_pct <= 100.0:
                raise RuntimeError(f"Curve '{curve_name}' has invalid MinFanSpeed")
            if not 0.0 <= max_fan_pct <= 100.0:
                raise RuntimeError(f"Curve '{curve_name}' has invalid MaxFanSpeed")

            curves[curve_name] = {
                "name": curve_name,
                "source_identifier": source_identifier,
                "idle_temp_c": idle_temp_c,
                "load_temp_c": load_temp_c,
                "min_fan_pct": min_fan_pct,
                "max_fan_pct": max_fan_pct,
            }

        if not curves:
            raise RuntimeError("No valid curves found in FanControl.FanCurves")

        self.curves_by_name = curves

    def _parse_custom_sensors(self, fan_control: Dict[str, object]) -> None:
        custom_sensors = fan_control.get("CustomSensors", [])
        if not isinstance(custom_sensors, list):
            raise RuntimeError("FanControl.CustomSensors must be an array when present")

        indexed: Dict[str, Dict[str, object]] = {}
        for raw_sensor in custom_sensors:
            if not isinstance(raw_sensor, dict):
                continue
            identifier = raw_sensor.get("Identifier")
            if not isinstance(identifier, str) or not identifier.strip():
                continue
            normalized = identifier.strip()
            if normalized in indexed:
                raise RuntimeError(f"Duplicate custom sensor identifier: {normalized}")
            indexed[normalized] = raw_sensor

        self.custom_sensors_by_id = indexed

    def _parse_active_controls(self, fan_control: Dict[str, object]) -> None:
        controls = fan_control.get("Controls")
        if not isinstance(controls, list):
            raise RuntimeError("Missing required array: FanControl.Controls")

        curve_to_ids: Dict[str, List[str]] = {}
        curve_to_pwms: Dict[str, List[str]] = {}

        for raw_control in controls:
            if not isinstance(raw_control, dict):
                continue
            if not bool(raw_control.get("Enable", False)):
                continue

            selected_curve = raw_control.get("SelectedFanCurve")
            if not isinstance(selected_curve, dict):
                continue

            curve_name = selected_curve.get("Name")
            if not isinstance(curve_name, str) or not curve_name.strip():
                continue
            curve_name = curve_name.strip()

            if curve_name not in self.curves_by_name:
                print(f"WARN: Control references unknown curve '{curve_name}', skipping")
                continue

            control_identifier = raw_control.get("Identifier")
            if not isinstance(control_identifier, str) or not control_identifier.strip():
                continue
            control_identifier = control_identifier.strip()

            pwm_path = self.control_map.get(control_identifier)
            if pwm_path is None:
                print(f"WARN: Missing HARDWARE_MAP control mapping for '{control_identifier}'")
                continue

            curve_to_ids.setdefault(curve_name, []).append(control_identifier)
            curve_to_pwms.setdefault(curve_name, []).append(pwm_path)

        if not curve_to_pwms:
            raise RuntimeError("No enabled controls with valid curve and hardware mapping found")

        self.active_curve_to_control_ids = {
            key: self._unique_preserve_order(value) for key, value in curve_to_ids.items()
        }
        self.active_curve_to_pwm_paths = {
            key: self._unique_preserve_order(value) for key, value in curve_to_pwms.items()
        }

        flat_paths: List[str] = []
        for pwm_paths in self.active_curve_to_pwm_paths.values():
            flat_paths.extend(pwm_paths)
        self.active_pwm_paths = self._unique_preserve_order(flat_paths)

    def _initialize_time_windows(self) -> None:
        self.time_window_samples = {}
        for identifier, custom_sensor in self.custom_sensors_by_id.items():
            if not self._is_time_sensor(custom_sensor):
                continue

            selected_time = custom_sensor.get("SelectedTime")
            if not isinstance(selected_time, (int, float)) or float(selected_time) <= 0.0:
                raise RuntimeError(f"Time sensor '{identifier}' has invalid SelectedTime")

            max_samples = max(1, int(round(float(selected_time) / self.poll_interval_seconds)))
            self.time_window_samples[identifier] = max_samples

    def _validate_active_curve_graphs(self) -> None:
        """Fail fast for invalid custom-sensor chains on active curves."""
        for curve_name in sorted(self.active_curve_to_pwm_paths):
            curve = self.curves_by_name[curve_name]
            source_identifier = str(curve["source_identifier"])
            self._validate_identifier_graph(source_identifier, [])

    def _validate_identifier_graph(self, identifier: str, stack: List[str]) -> None:
        if identifier in stack:
            cycle = " -> ".join(stack + [identifier])
            raise RuntimeError(f"Cycle detected in sensor graph: {cycle}")

        custom_sensor = self.custom_sensors_by_id.get(identifier)
        if custom_sensor is None:
            self._validate_base_identifier(identifier)
            return

        if self._is_mix_sensor(custom_sensor):
            selected_sensors = custom_sensor.get("SelectedSensors")
            if not isinstance(selected_sensors, list) or not selected_sensors:
                raise RuntimeError(f"Custom sensor '{identifier}' has no SelectedSensors")
            for child in selected_sensors:
                if not isinstance(child, dict):
                    raise RuntimeError(
                        f"Custom sensor '{identifier}' has invalid SelectedSensors entry"
                    )
                child_identifier = child.get("Identifier")
                if not isinstance(child_identifier, str) or not child_identifier.strip():
                    raise RuntimeError(
                        f"Custom sensor '{identifier}' has invalid child Identifier"
                    )
                self._validate_identifier_graph(child_identifier.strip(), stack + [identifier])
            return

        if self._is_time_sensor(custom_sensor):
            source_identifier = self._extract_selected_temp_source_identifier(custom_sensor)
            self._validate_identifier_graph(source_identifier, stack + [identifier])
            return

        if "SelectedTempSource" in custom_sensor:
            source_identifier = self._extract_selected_temp_source_identifier(custom_sensor)
            self._validate_identifier_graph(source_identifier, stack + [identifier])
            return

        raise RuntimeError(
            f"Unsupported custom sensor type for '{identifier}' (expected Time or Mix)"
        )

    def _validate_base_identifier(self, identifier: str) -> None:
        if self._is_cpu_identifier(identifier):
            return
        if self._is_gpu_identifier(identifier):
            return
        if self._is_nvme_identifier(identifier):
            self._parse_nvme_identifier(identifier)
            return
        if self._is_lpc_temp_identifier(identifier):
            self._parse_lpc_temperature_identifier(identifier)
            return
        if identifier.startswith("/sys/class/hwmon/"):
            return

        raise RuntimeError(f"Unsupported base sensor identifier: '{identifier}'")

    def _describe_curve_graph(self, curve_name: str) -> List[str]:
        """Return ordered list of custom sensors used by a curve source chain."""
        curve = self.curves_by_name[curve_name]
        source_identifier = str(curve["source_identifier"])
        visited: Set[str] = set()
        ordered: List[str] = []

        def walk(identifier: str) -> None:
            custom_sensor = self.custom_sensors_by_id.get(identifier)
            if custom_sensor is None:
                return
            if identifier in visited:
                return

            visited.add(identifier)
            ordered.append(identifier)

            if self._is_mix_sensor(custom_sensor):
                selected_sensors = custom_sensor.get("SelectedSensors")
                if isinstance(selected_sensors, list):
                    for child in selected_sensors:
                        if isinstance(child, dict):
                            child_identifier = child.get("Identifier")
                            if isinstance(child_identifier, str) and child_identifier.strip():
                                walk(child_identifier.strip())
                return

            if "SelectedTempSource" in custom_sensor:
                source = custom_sensor.get("SelectedTempSource")
                if isinstance(source, dict):
                    source_identifier = source.get("Identifier")
                    if isinstance(source_identifier, str) and source_identifier.strip():
                        walk(source_identifier.strip())

        walk(source_identifier)
        return ordered

    def _require_str_field(self, data: Dict[str, object], key: str) -> str:
        raw = data.get(key)
        if not isinstance(raw, str):
            raise RuntimeError(f"Missing or invalid string field '{key}'")
        value = raw.strip()
        if not value:
            raise RuntimeError(f"Field '{key}' cannot be empty")
        return value

    def _require_float_field(self, data: Dict[str, object], key: str) -> float:
        raw = data.get(key)
        if isinstance(raw, (int, float)):
            return float(raw)
        if isinstance(raw, str):
            value = raw.strip()
            if not value:
                raise RuntimeError(f"Field '{key}' cannot be empty")
            try:
                return float(value)
            except ValueError as exc:
                raise RuntimeError(f"Field '{key}' is not numeric: {raw}") from exc
        raise RuntimeError(f"Missing or invalid numeric field '{key}'")

    def _unique_preserve_order(self, values: List[str]) -> List[str]:
        return list(dict.fromkeys(values))

    def _is_cpu_identifier(self, identifier: str) -> bool:
        return identifier.startswith("/amdcpu/")

    def _is_gpu_identifier(self, identifier: str) -> bool:
        return "NVApiWrapper" in identifier and "/sensor/" in identifier

    def _is_nvme_identifier(self, identifier: str) -> bool:
        return identifier.startswith("/nvme/")

    def _is_lpc_temp_identifier(self, identifier: str) -> bool:
        return identifier.startswith("/lpc/nct6797d/temperature/")

    def _is_time_sensor(self, custom_sensor: Dict[str, object]) -> bool:
        identifier = custom_sensor.get("Identifier")
        return isinstance(identifier, str) and identifier.startswith("Time/")

    def _is_mix_sensor(self, custom_sensor: Dict[str, object]) -> bool:
        return "SelectedMixFunction" in custom_sensor and "SelectedSensors" in custom_sensor

    def _extract_selected_temp_source_identifier(self, node: Dict[str, object]) -> str:
        selected_temp_source = node.get("SelectedTempSource")
        if not isinstance(selected_temp_source, dict):
            raise RuntimeError("Node is missing SelectedTempSource")
        identifier = selected_temp_source.get("Identifier")
        if not isinstance(identifier, str) or not identifier.strip():
            raise RuntimeError("Node has invalid SelectedTempSource.Identifier")
        return identifier.strip()

    def _read_hwmon_temp_path(self, temp_path: str) -> float:
        with open(temp_path, "r", encoding="utf-8") as file_handle:
            raw = file_handle.read().strip()
        return float(raw) / 1000.0

    def _read_cpu_temp_max(self) -> float:
        cache_key = "__cpu_max__"
        cached = self._cycle_cache.get(cache_key)
        if cached is not None:
            return cached

        values: List[float] = []
        for temp_path in self.cpu_temp_paths:
            try:
                values.append(self._read_hwmon_temp_path(temp_path))
            except (OSError, ValueError) as exc:
                print(f"WARN: Failed reading CPU temp '{temp_path}': {exc}")

        if not values:
            raise RuntimeError("No readable CPU temperature values from CPU_TEMP_PATHS")

        value = max(values)
        self._cycle_cache[cache_key] = value
        return value

    def get_gpu_temp(self) -> float:
        cache_key = "__gpu_max__"
        cached = self._cycle_cache.get(cache_key)
        if cached is not None:
            return cached

        if self.gpu_method != "nvidia_smi_query":
            raise RuntimeError(f"Unsupported GPU_METHOD '{self.gpu_method}'")

        try:
            output = subprocess.check_output(
                NVIDIA_SMI_QUERY_CMD,
                text=True,
                timeout=3.0,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("nvidia-smi not found in PATH") from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"nvidia-smi timed out: {exc}") from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f"nvidia-smi failed: {exc}") from exc

        values: List[float] = []
        for line in output.splitlines():
            text_line = line.strip()
            if not text_line:
                continue
            first_field = text_line.split(",", maxsplit=1)[0].strip()
            try:
                values.append(float(first_field))
            except ValueError as exc:
                raise RuntimeError(f"Unable to parse GPU temp from '{text_line}'") from exc

        if not values:
            raise RuntimeError("nvidia-smi returned no GPU temperature values")

        value = max(values)
        self._cycle_cache[cache_key] = value
        return value

    def _discover_nvme_temp_paths(self) -> None:
        """Map /nvme/N/temperature/M to hwmon temp(M+1)_input by NVMe index."""
        mapping: Dict[int, Dict[int, str]] = {}
        hwmon_root = "/sys/class/hwmon"

        try:
            entries = sorted(os.listdir(hwmon_root))
        except OSError:
            self.nvme_paths_by_index = {}
            return

        for entry in entries:
            hwmon_dir = os.path.join(hwmon_root, entry)
            if not os.path.isdir(hwmon_dir):
                continue

            name_path = os.path.join(hwmon_dir, "name")
            try:
                with open(name_path, "r", encoding="utf-8") as file_handle:
                    hwmon_name = file_handle.read().strip().lower()
            except OSError:
                continue

            if hwmon_name != "nvme":
                continue

            nvme_index = self._extract_nvme_index_from_hwmon_dir(hwmon_dir)
            if nvme_index is None:
                continue

            temp_index_map: Dict[int, str] = {}
            try:
                file_names = sorted(os.listdir(hwmon_dir))
            except OSError:
                continue

            for file_name in file_names:
                if not file_name.startswith("temp") or not file_name.endswith("_input"):
                    continue

                middle = file_name[len("temp") : -len("_input")]
                if not middle.isdigit():
                    continue

                # Linux temp1_input corresponds to Windows /temperature/0.
                logical_temp_index = int(middle) - 1
                if logical_temp_index < 0:
                    continue

                temp_path = os.path.join(hwmon_dir, file_name)
                if os.path.isfile(temp_path):
                    temp_index_map[logical_temp_index] = temp_path

            if temp_index_map:
                mapping[nvme_index] = temp_index_map

        self.nvme_paths_by_index = mapping

    def _extract_nvme_index_from_hwmon_dir(self, hwmon_dir: str) -> Optional[int]:
        real = os.path.realpath(hwmon_dir)
        parts = real.split("/")

        for idx, part in enumerate(parts):
            if part != "nvme":
                continue
            if idx + 1 >= len(parts):
                continue

            nvme_name = parts[idx + 1]
            if not nvme_name.startswith("nvme"):
                continue

            suffix = nvme_name[len("nvme") :]
            if suffix.isdigit():
                return int(suffix)

        return None

    def _parse_nvme_identifier(self, identifier: str) -> Tuple[int, int]:
        parts = identifier.strip("/").split("/")
        if len(parts) != 4 or parts[0] != "nvme" or parts[2] != "temperature":
            raise RuntimeError(f"Invalid NVMe identifier format: '{identifier}'")
        if not parts[1].isdigit() or not parts[3].isdigit():
            raise RuntimeError(f"Invalid NVMe identifier values: '{identifier}'")
        return int(parts[1]), int(parts[3])

    def _read_nvme_identifier(self, identifier: str) -> float:
        cache_key = f"__nvme__:{identifier}"
        cached = self._cycle_cache.get(cache_key)
        if cached is not None:
            return cached

        nvme_index, temp_index = self._parse_nvme_identifier(identifier)

        temp_path = self.nvme_paths_by_index.get(nvme_index, {}).get(temp_index)
        if temp_path is None:
            # Refresh once in case hwmon enumeration changed at runtime.
            self._discover_nvme_temp_paths()
            temp_path = self.nvme_paths_by_index.get(nvme_index, {}).get(temp_index)

        if temp_path is None:
            raise RuntimeError(
                f"NVMe sensor not available for identifier '{identifier}' "
                f"(nvme{nvme_index}, temperature/{temp_index})"
            )

        value = self._read_hwmon_temp_path(temp_path)
        self._cycle_cache[cache_key] = value
        return value

    def _parse_lpc_temperature_identifier(self, identifier: str) -> int:
        suffix = identifier.rsplit("/", maxsplit=1)[-1]
        if not suffix.isdigit():
            raise RuntimeError(f"Invalid LPC temperature identifier: '{identifier}'")
        return int(suffix)

    def _read_lpc_temperature_identifier(self, identifier: str) -> float:
        cache_key = f"__lpc__:{identifier}"
        cached = self._cycle_cache.get(cache_key)
        if cached is not None:
            return cached

        windows_index = self._parse_lpc_temperature_identifier(identifier)

        # Windows /temperature/N maps to Linux temp(N+1)_input.
        linux_index = windows_index + 1
        temp_path = os.path.join(self.nct_hwmon_dir, f"temp{linux_index}_input")
        value = self._read_hwmon_temp_path(temp_path)

        self._cycle_cache[cache_key] = value
        return value

    def _resolve_base_identifier(self, identifier: str) -> float:
        if self._is_cpu_identifier(identifier):
            return self._read_cpu_temp_max()
        if self._is_gpu_identifier(identifier):
            return self.get_gpu_temp()
        if self._is_nvme_identifier(identifier):
            return self._read_nvme_identifier(identifier)
        if self._is_lpc_temp_identifier(identifier):
            return self._read_lpc_temperature_identifier(identifier)
        if identifier.startswith("/sys/class/hwmon/"):
            return self._read_hwmon_temp_path(identifier)

        raise RuntimeError(f"Unsupported base sensor identifier '{identifier}'")

    def _resolve_sensor_identifier(self, identifier: str, stack: List[str]) -> float:
        cached = self._cycle_cache.get(identifier)
        if cached is not None:
            return cached

        if identifier in stack:
            cycle = " -> ".join(stack + [identifier])
            raise RuntimeError(f"Cycle detected in sensor graph: {cycle}")

        custom_sensor = self.custom_sensors_by_id.get(identifier)
        if custom_sensor is None:
            value = self._resolve_base_identifier(identifier)
            self._cycle_cache[identifier] = value
            return value

        if self._is_mix_sensor(custom_sensor):
            value = self._evaluate_mix_sensor(identifier, custom_sensor, stack)
        elif self._is_time_sensor(custom_sensor):
            value = self._evaluate_time_sensor(identifier, custom_sensor, stack)
        elif "SelectedTempSource" in custom_sensor:
            source_identifier = self._extract_selected_temp_source_identifier(custom_sensor)
            value = self._resolve_sensor_identifier(source_identifier, stack + [identifier])
        else:
            raise RuntimeError(f"Unsupported custom sensor type for '{identifier}'")

        self._cycle_cache[identifier] = value
        return value

    def _evaluate_mix_sensor(
        self,
        identifier: str,
        custom_sensor: Dict[str, object],
        stack: List[str],
    ) -> float:
        selected_mix_function = custom_sensor.get("SelectedMixFunction")
        if not isinstance(selected_mix_function, (int, float)):
            raise RuntimeError(f"Custom sensor '{identifier}' missing SelectedMixFunction")

        mix_function = int(selected_mix_function)
        if mix_function not in (1, 2):
            raise RuntimeError(
                f"Custom sensor '{identifier}' has unsupported SelectedMixFunction={mix_function}"
            )

        allow_missing = bool(custom_sensor.get("AllowMissingSensor", False))

        selected_sensors = custom_sensor.get("SelectedSensors")
        if not isinstance(selected_sensors, list) or not selected_sensors:
            raise RuntimeError(f"Custom sensor '{identifier}' has empty SelectedSensors")

        child_values: List[float] = []
        for raw_child in selected_sensors:
            if not isinstance(raw_child, dict):
                raise RuntimeError(f"Custom sensor '{identifier}' has invalid child node")

            child_identifier = raw_child.get("Identifier")
            if not isinstance(child_identifier, str) or not child_identifier.strip():
                raise RuntimeError(f"Custom sensor '{identifier}' has invalid child Identifier")

            normalized_child = child_identifier.strip()
            try:
                child_value = self._resolve_sensor_identifier(
                    normalized_child,
                    stack + [identifier],
                )
                child_values.append(child_value)
            except RuntimeError as exc:
                if allow_missing:
                    print(
                        f"WARN: {identifier} skipping child '{normalized_child}' due to: {exc}"
                    )
                    continue
                raise

        if not child_values:
            raise RuntimeError(f"Custom sensor '{identifier}' has no readable child sensors")

        if mix_function == 1:
            return max(child_values)
        return min(child_values)

    def _evaluate_time_sensor(
        self,
        identifier: str,
        custom_sensor: Dict[str, object],
        stack: List[str],
    ) -> float:
        source_identifier = self._extract_selected_temp_source_identifier(custom_sensor)
        source_value = self._resolve_sensor_identifier(source_identifier, stack + [identifier])

        max_samples = self.time_window_samples.get(identifier)
        if max_samples is None:
            raise RuntimeError(f"Time sensor '{identifier}' has no sample-window configuration")

        window = self.time_windows.get(identifier)
        if window is None:
            window = deque()
            self.time_windows[identifier] = window
            # Cold-start stabilization: pre-fill window with first sample.
            self.time_window_sums[identifier] = source_value * max_samples
            for _ in range(max_samples):
                window.append(source_value)

        if len(window) >= max_samples:
            removed = window.popleft()
            self.time_window_sums[identifier] -= removed

        window.append(source_value)
        self.time_window_sums[identifier] += source_value

        return self.time_window_sums[identifier] / float(len(window))

    def _interpolate_percent(self, curve: Dict[str, object], source_temp_c: float) -> float:
        idle_temp_c = float(curve["idle_temp_c"])
        load_temp_c = float(curve["load_temp_c"])
        min_fan_pct = float(curve["min_fan_pct"])
        max_fan_pct = float(curve["max_fan_pct"])

        if source_temp_c <= idle_temp_c:
            return min_fan_pct
        if source_temp_c >= load_temp_c:
            return max_fan_pct

        ratio = (source_temp_c - idle_temp_c) / (load_temp_c - idle_temp_c)
        return min_fan_pct + ratio * (max_fan_pct - min_fan_pct)

    def _percent_to_pwm(self, percent: float) -> int:
        clamped = max(0.0, min(100.0, percent))
        return int(round((clamped / 100.0) * 255.0))

    def _pwm_enable_path(self, pwm_path: str) -> str:
        base = os.path.dirname(pwm_path)
        name = os.path.basename(pwm_path)
        return os.path.join(base, f"{name}_enable")

    def _set_manual_mode(self, pwm_path: str) -> bool:
        enable_path = self._pwm_enable_path(pwm_path)
        if not os.path.exists(enable_path):
            print(f"WARN: PWM enable path missing: {enable_path}")
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
        except OSError as exc:
            print(f"ERROR: Failed writing PWM '{pwm_path}': {exc}")
            return False

    def _preflight_checks(self) -> bool:
        if os.geteuid() != 0:
            print("ERROR: Daemon must run as root to write PWM sysfs files")
            return False

        preflight_ok = True
        for pwm_path in self.active_pwm_paths:
            if not os.path.exists(pwm_path):
                print(f"ERROR: PWM path not found: {pwm_path}")
                preflight_ok = False
                continue

            self._set_manual_mode(pwm_path)
            if not os.access(pwm_path, os.W_OK):
                print(f"ERROR: PWM path is not writable: {pwm_path}")
                preflight_ok = False

        return preflight_ok

    def _handle_stop_signal(self, signum: int, _frame: object) -> None:
        print(f"INFO: Received signal {signum}; requesting shutdown")
        self._running = False

    def install_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_stop_signal)
        signal.signal(signal.SIGTERM, self._handle_stop_signal)

    def run(self) -> int:
        print("INFO: x570-thermal-daemon starting")
        print(f"INFO: Config path: {self.config_path}")
        print(f"INFO: Poll interval={self.poll_interval_seconds:.1f}s")
        print(f"INFO: Active curves: {sorted(self.active_curve_to_pwm_paths.keys())}")

        for curve_name in sorted(self.active_curve_to_pwm_paths):
            curve = self.curves_by_name[curve_name]
            source_identifier = str(curve["source_identifier"])
            chain = self._describe_curve_graph(curve_name)
            print(
                "INFO: "
                f"curve={curve_name} source={source_identifier} "
                f"pwms={self.active_curve_to_pwm_paths[curve_name]}"
            )
            if chain:
                print(f"INFO: curve={curve_name} custom_chain={chain}")

        if not self._preflight_checks():
            print("ERROR: Preflight checks failed; aborting daemon startup")
            return 1

        try:
            while self._running:
                cycle_start = time.time()
                self._cycle_cache = {}

                for curve_name in sorted(self.active_curve_to_pwm_paths):
                    curve = self.curves_by_name[curve_name]
                    source_identifier = str(curve["source_identifier"])

                    try:
                        source_temp = self._resolve_sensor_identifier(source_identifier, [])
                        old_error = self._curve_error_state.pop(curve_name, None)
                        if old_error is not None:
                            print(f"INFO: curve={curve_name} sensor chain recovered")
                    except RuntimeError as exc:
                        error_text = str(exc)
                        if self._curve_error_state.get(curve_name) != error_text:
                            print(
                                "WARN: "
                                f"curve={curve_name} sensor chain failure: {error_text}; "
                                "falling back to LoadTemperature"
                            )
                        self._curve_error_state[curve_name] = error_text
                        source_temp = float(curve["load_temp_c"])

                    target_percent = self._interpolate_percent(curve, source_temp)
                    target_pwm = self._percent_to_pwm(target_percent)

                    pwm_paths = self.active_curve_to_pwm_paths[curve_name]
                    writes_ok = 0
                    for pwm_path in pwm_paths:
                        if self._write_pwm(pwm_path, target_pwm):
                            writes_ok += 1

                    print(
                        "INFO: "
                        f"curve={curve_name} "
                        f"source={source_temp:.2f}C "
                        f"target={target_percent:.1f}% ({target_pwm}/255) "
                        f"writes={writes_ok}/{len(pwm_paths)}"
                    )

                elapsed = time.time() - cycle_start
                sleep_s = self.poll_interval_seconds - elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)
        finally:
            self.shutdown()
        return 0

    def shutdown(self) -> None:
        if self._shutdown_done:
            return

        self._shutdown_done = True
        print("INFO: Writing failsafe PWM=255 to all active mapped channels")
        for pwm_path in self.active_pwm_paths:
            self._set_manual_mode(pwm_path)
            self._write_pwm(pwm_path, 255)
        print("INFO: x570-thermal-daemon stopped")


def main() -> int:
    config_path = os.environ.get("X570_THERMAL_CONFIG", "userConfig.json")

    poll_env = os.environ.get("X570_THERMAL_POLL_INTERVAL", "1.0")
    try:
        poll_interval_seconds = float(poll_env)
    except ValueError as exc:
        print(f"ERROR: Invalid X570_THERMAL_POLL_INTERVAL '{poll_env}': {exc}")
        return 1

    try:
        daemon = ThermalDaemon(
            config_path=config_path,
            poll_interval_seconds=poll_interval_seconds,
        )
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: Failed initializing daemon: {exc}")
        return 1

    daemon.install_signal_handlers()

    try:
        return daemon.run()
    except Exception as exc:  # pylint: disable=broad-except
        print(f"ERROR: Unhandled runtime exception: {exc}")
        daemon.shutdown()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
