#!/usr/bin/env bash
set -euo pipefail
project_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$project_directory/raspberry_pi/launch.sh" --task-console --web-port 8088 "$@"
