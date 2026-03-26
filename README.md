# x570-thermal-daemon

`x570-thermal-daemon` is a lightweight Python 3 daemon that bridges a Windows FanControl curve (`userConfig.json`) to Linux `hwmon` PWM outputs.

It is designed for custom liquid loops where fan speed should react to a smoothed thermal signal instead of instant CPU/GPU spikes.

## What It Does

On each poll cycle, the daemon:

1. Reads CPU temperature from `hwmon` (millidegree Celsius -> Celsius) and GPU temperature via `nvidia-smi`.
2. Computes `current_max = max(cpu_temp, gpu_temp)`.
3. Maintains a rolling time-average window to simulate virtual water temperature.
4. Applies linear interpolation using curve points from `userConfig.json`:
   - `IdleTemperature` -> `MinFanSpeed`
   - `LoadTemperature` -> `MaxFanSpeed`
5. Converts percent to `0..255` PWM and writes to mapped fan channels.

Hardware mapping is loaded from a separate JSON profile file (default: `hardware_map_msi_x570_unify.json`).

## Safety Behavior

- Requires root (`sudo`) for sysfs writes.
- Tries to set `pwmX_enable` to `1` (manual mode) during preflight.
- On `SIGINT`/`SIGTERM`, writes `255` (100%) to all managed PWM channels.
- If all sensor reads fail, falls back to last valid reading, then to `LoadTemperature`.
- Stops after 10 consecutive cycles with zero successful PWM writes.

## Requirements

- Linux kernel with `hwmon` support for your Super I/O (`nct6775` for Nuvoton NCT6797D).
- Python 3.8+.
- Root privileges.

## Quick Start

1. Ensure the Nuvoton driver is loaded:

```bash
sudo modprobe nct6775
```

2. Verify your hwmon devices and PWM files:

```bash
for h in /sys/class/hwmon/hwmon*; do
  echo "== $h =="
  sudo cat "$h/name" 2>/dev/null || true
  ls "$h"/pwm* 2>/dev/null || true
done
```

3. Run daemon:

```bash
sudo python3 x570_thermal_daemon.py
```

## Persist nct6775 Across Reboots

```bash
echo nct6775 | sudo tee /etc/modules-load.d/nct6775.conf
```

Reboot once to validate auto-load:

```bash
lsmod | grep nct6775
```

## Configuration

### 1) FanControl JSON Input

Default file is `userConfig.json` in this repo. The daemon reads:

- `FanControl.FanCurves[]`
- Curve name default: `Auto Water Cooling`
- Required fields from that curve:
  - `IdleTemperature`
  - `LoadTemperature`
  - `MinFanSpeed`
  - `MaxFanSpeed`

### 2) Linux Hardware Map

Default map file is `hardware_map_msi_x570_unify.json` (MSI X570 Unify profile).

Edit that JSON file if needed.

- `temperature_sensors.CPU_TEMP_PATH`: CPU temp hwmon path.
- `temperature_sensors.GPU_METHOD`: currently `nvidia_smi_query`.
- `controls`: FanControl control identifier -> Linux PWM sysfs path.

The daemon requires this map file at startup (no built-in legacy map fallback).

Behavior notes:

- Only controls assigned to the selected curve (default `Auto Water Cooling`) are actively driven.
- All mapped controls are forced to 100% on daemon shutdown as a failsafe.

### 3) Environment Overrides

- `X570_THERMAL_CONFIG` (default: `userConfig.json`)
- `X570_THERMAL_MAP` (default: `hardware_map_msi_x570_unify.json`)
- `X570_THERMAL_CURVE` (default: `Auto Water Cooling`)
- `X570_THERMAL_WINDOW_SECONDS` (default: `180`)
- `X570_THERMAL_POLL_INTERVAL` (default: `1.0`)

Example:

```bash
sudo X570_THERMAL_MAP=hardware_map_msi_x570_unify.json X570_THERMAL_WINDOW_SECONDS=240 X570_THERMAL_POLL_INTERVAL=1.5 python3 x570_thermal_daemon.py
```

## Troubleshooting

### `PWM path not found`

- `nct6775` may not be loaded, or your board exposes different channels.
- Run `sudo modprobe nct6775` and re-check `/sys/class/hwmon/hwmon*/pwm*`.

### `Permission denied` on `pwmX`

- Run daemon with `sudo`.
- Confirm `pwmX_enable` exists and is writable.

### No suitable sensor path

- Ensure CPU hwmon path exists and is readable.
- Ensure `nvidia-smi` works for GPU temperature queries.
- Inspect available CPU sensor files:

```bash
for h in /sys/class/hwmon/hwmon*; do
  echo "== $h =="
  cat "$h/name" 2>/dev/null || true
  ls "$h"/temp*_input 2>/dev/null || true
done
```

## Optional: systemd Service

Use the bundled unit file from this repo:

```bash
sudo cp /home/ye/Workspace/x570-thermal-daemon/x570-thermal-daemon.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now x570-thermal-daemon.service
sudo systemctl status x570-thermal-daemon.service
```

Recommended for real-time logs: run Python unbuffered in the unit by setting:

```ini
ExecStart=/usr/bin/python3 -u /home/ye/Workspace/x570-thermal-daemon/x570_thermal_daemon.py
```

Important: `ExecStart=...` is a `systemd` unit directive, not a shell command.

Service lifecycle notes:

- `enable` only controls boot-time autostart.
- To apply code or unit-file changes now, run:

```bash
sudo systemctl daemon-reload
sudo systemctl restart x570-thermal-daemon.service
sudo systemctl status x570-thermal-daemon.service
sudo journalctl -u x570-thermal-daemon.service -f
```

## Development

```bash
python3 -m py_compile x570_thermal_daemon.py
pylint x570_thermal_daemon.py
```

If you use VS Code/Pylance, this repo includes `pyrightconfig.json` with relaxed checking (`basic`).

## License

This project is licensed under the MIT License. See `LICENSE`.

Attribution requirement: if you redistribute or reuse this project (or substantial portions of it), you must keep the original copyright and license notice so the source remains credited.
