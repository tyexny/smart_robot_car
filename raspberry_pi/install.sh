#!/usr/bin/env bash
# Run with: bash install.sh [--venv /path/to/existing/or/new/venv]
set -euo pipefail
task_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
task_venv="${VIRTUAL_ENV:-$task_directory/.venv}"
if [[ ${1:-} == --venv && $# == 2 ]]; then
    task_venv=$2
elif [[ $# != 0 ]]; then
    printf 'Usage: bash install.sh [--venv PATH]\n' >&2
    exit 2
fi
if [[ ! -x "$task_venv/bin/python" ]]; then
    python3 -m venv "$task_venv"
fi
"$task_venv/bin/python" -m pip --disable-pip-version-check install --upgrade pip
"$task_venv/bin/python" -m pip --disable-pip-version-check install -r "$task_directory/requirements.txt"
"$task_venv/bin/python" -c 'import cv2, lap, ncnn, numpy, scipy, ultralytics; assert ultralytics.__version__ == "8.4.159"; print("Tracking dependencies ready")'
printf 'Ready. Activate %s/bin/activate, then: bash launch.sh --preview\n' "$task_venv"
