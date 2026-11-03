#!/usr/bin/env bash
set -euo pipefail
project_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$project_directory/raspberry_pi/launch.sh" \
  --vlm-url "${LOCAL_VLM_URL:-http://127.0.0.1:8080/v1/chat/completions}" \
  --task-console --web-port 8088 "$@"
