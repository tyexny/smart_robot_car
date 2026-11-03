#!/usr/bin/env bash
# Capture Pi 5 supply voltage and load while YOLO and VLM run together.
set -u

task6_health_log=${1:-"$HOME/task6_health.log"}
printf 'Task 6 health log: %s\nPress Ctrl+C to stop.\n' "$task6_health_log" | tee -a "$task6_health_log"

while true; do
    {
        printf '\n%s\n' "$(date -Is)"
        vcgencmd pmic_read_adc EXT5V_V
        vcgencmd get_throttled
        vcgencmd measure_temp
        awk '/MemAvailable:|SwapFree:/ { print $1, $2, $3 }' /proc/meminfo
        ps -eo comm,pid,%cpu,rss --sort=-rss | head -n 6
    } | tee -a "$task6_health_log"
    sync -d "$task6_health_log" 2>/dev/null || true
    sleep 1
done
