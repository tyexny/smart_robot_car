#!/usr/bin/env bash
set -euo pipefail
project_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$project_directory/start_local_pi.sh" "$@"
