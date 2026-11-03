#!/usr/bin/env bash
# Run on the Raspberry Pi: bash install_thermal.sh
set -euo pipefail
thermal_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if ! command -v g++ >/dev/null || ! command -v make >/dev/null; then
    printf 'Install build tools first: sudo apt install build-essential\n' >&2
    exit 1
fi
make -B -C "$thermal_root/mlx90640"
printf '\nMLX90640 native library built for this Pi.\n'
printf 'Sensor I2C is now on FPGA: SDA E13, SCL E14, 3.3V supply and common GND.\n'
printf 'Pi GPIO I2C is not used. Keep the existing Pi-FPGA Ethernet connection.\n'
printf 'Check: python3 "%s/thermal_monitor.py" --check\n' "$thermal_root"
