#!/usr/bin/env bash
set -euo pipefail
task_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ -n ${VIRTUAL_ENV:-} && -x "$VIRTUAL_ENV/bin/python" ]]; then
    task_python="$VIRTUAL_ENV/bin/python"
elif [[ -x "$task_directory/.venv/bin/python" ]]; then
    task_python="$task_directory/.venv/bin/python"
else
    task_python=python3
fi
exec "$task_python" "$task_directory/udp_yolo_receiver.py" "$@"
