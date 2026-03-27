# x570-thermal-daemon

`x570-thermal-daemon` is a lightweight Python 3 daemon that mirrors a Windows FanControl profile (`userConfig.json`) and drives Linux `hwmon` PWM outputs.

It is designed for custom liquid loops where fan speed should follow a smoothed, profile-driven thermal signal rather than instantaneous spikes.

## What It Does

On each poll cycle, the daemon:

1. Loads the selected curve (`Auto Water Cooling` by default).
2. Resolves the curve's `SelectedTempSource.Identifier` from `FanControl.CustomSensors`.
3. Evaluates the source sensor graph recursively:
   - Mix function `1` -> `MAX`
   - Mix function `2` -> `MIN`
   - `Time/*` sensors -> rolling average using each node's `SelectedTime`.
4. Reads base temperatures from:
   - CPU: max of configured CPU hwmon paths (`CPU_TEMP_PATHS`)
   - GPU: `nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader`
5. Interpolates the resolved source temperature against curve points:
   - `IdleTemperature` -> `MinFanSpeed`
   - `LoadTemperature` -> `MaxFanSpeed`
6. Converts percent to `0..255` and writes PWM to controls assigned to the same curve.

## Safety Behavior

- Requires root (`sudo`) for sysfs writes.
- Attempts `pwmX_enable=1` (manual mode) before PWM writes.
- Catches `SIGINT`/`SIGTERM` and writes `255` (100%) to all mapped PWM channels on exit.
- If sensor evaluation fails at runtime, uses last valid source temperature; if none exists, falls back to curve `LoadTemperature`.
- Stops after 10 consecutive cycles with zero successful PWM writes.

## Requirements

- Linux kernel with `nct6775` support for Nuvoton NCT6797D.
- Python 3.8+.
- Root privileges.
- `nvidia-smi` available in PATH.

## Quick Start

1. Ensure Nuvoton driver is loaded:

```bash
sudo modprobe nct6775
```

2. Verify hwmon devices and PWM files:

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

Validate after reboot:

```bash
lsmod | grep nct6775
```

## Configuration

### 1) FanControl JSON Input

Default file: `userConfig.json`

The daemon uses:

- `FanControl.FanCurves[]` for linear points and `SelectedTempSource`
- `FanControl.CustomSensors[]` for recursive sensor graph evaluation
- `FanControl.Controls[]` to select active controls using the same curve name

Default curve name: `Auto Water Cooling`

### 2) Linux Hardware Map

Default map: `hardware_map_msi_x570_unify.json`

Schema:

- `temperature_sensors.CPU_TEMP_PATHS`: list of CPU hwmon paths (daemon takes max)
- `temperature_sensors.GPU_METHOD`: currently `nvidia_smi_query`
- `controls`: `FanControl` control identifier -> Linux PWM sysfs path

Current host mapping in repo uses:

- CPU: `/sys/class/hwmon/hwmon4/temp1_input`, `temp3_input`, `temp4_input`
- Nuvoton PWM: `/sys/class/hwmon/hwmon3/pwm1..pwm7`

### 3) Environment Overrides

- `X570_THERMAL_CONFIG` (default: `userConfig.json`)
- `X570_THERMAL_MAP` (default: `hardware_map_msi_x570_unify.json`)
- `X570_THERMAL_CURVE` (default: `Auto Water Cooling`)
- `X570_THERMAL_POLL_INTERVAL` (default: `1.0`)

Example:

```bash
sudo X570_THERMAL_MAP=hardware_map_msi_x570_unify.json X570_THERMAL_POLL_INTERVAL=1.0 python3 x570_thermal_daemon.py
```

## Troubleshooting

### `No enabled controls mapped to fan curve ...`

- Ensure target controls are `Enable=true` and `SelectedFanCurve.Name` matches `X570_THERMAL_CURVE`.

### `Unsupported base sensor identifier ...`

- The selected source chain references identifiers the daemon cannot map.
- Supported base identifiers in graph:
  - `/amdcpu/...` -> CPU max from `CPU_TEMP_PATHS`
  - `NVApiWrapper/.../sensor/...` -> GPU via `nvidia-smi`
  - direct `/sys/class/hwmon/...` temperature paths

### `Permission denied` on `pwmX`

- Run daemon with `sudo`.
- Confirm `pwmX_enable` exists and is writable.

## Optional: systemd Service

Use bundled unit file:

```bash
sudo cp /home/ye/Workspace/x570-thermal-daemon/x570-thermal-daemon.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now x570-thermal-daemon.service
sudo systemctl status x570-thermal-daemon.service
```

For real-time logs, set unbuffered Python in service:

```ini
ExecStart=/usr/bin/python3 -u /home/ye/Workspace/x570-thermal-daemon/x570_thermal_daemon.py
```

Apply updates immediately after code/unit changes:

```bash
sudo systemctl daemon-reload
sudo systemctl restart x570-thermal-daemon.service
sudo journalctl -u x570-thermal-daemon.service -f
```

## Development

```bash
python3 -m py_compile x570_thermal_daemon.py
pylint x570_thermal_daemon.py
```

## License

MIT License. See `LICENSE`.

Attribution is required by retaining the copyright/license notice in redistributed copies.
