# x570-thermal-daemon 

A lightweight, hardware-level Python daemon for Linux that handles thermal inertia in custom liquid cooling loops without dedicated water temperature sensors. Tailored for the MSI MEG X570 Unify (Nuvoton NCT6775 Super I/O).

## 🚀 The Problem
Standard Linux fan control utilities (`fancontrol`, `lm-sensors`) map CPU/GPU temperatures directly to fan PWM signals. In a custom liquid cooling loop, this 1:1 mapping causes aggressive, unnecessary fan ramping due to sudden CPU/GPU load spikes, completely ignoring the high specific heat capacity (thermal mass) of the liquid.

## 💡 The Solution
This daemon introduces a **Time-Averaged Maximum Temperature Algorithm**. 
1. It continuously polls both CPU (`k10temp`) and GPU (`amdgpu`/`nvidia`) hardware monitors.
2. Extracts the `max(T_cpu, T_gpu)`.
3. Applies a moving average over a configurable time window (e.g., 120 seconds) to simulate the actual water temperature.
4. Writes the mapped PWM value directly to the Nuvoton Super I/O controller via `sysfs`.

## ⚙️ Prerequisites
- Linux Kernel 5.15+ (for native `nct6775` module support)
- Python 3.8+
- Root privileges (required for writing to `/sys/class/hwmon/`)

## 🛠️ Installation & Setup (Systemd)

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/yourusername/x570-thermal-daemon.git](https://github.com/yourusername/x570-thermal-daemon.git)
   cd x570-thermal-daemon