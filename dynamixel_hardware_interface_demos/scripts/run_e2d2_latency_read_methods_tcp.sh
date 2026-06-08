#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export TRANSPORT=tcp
exec "${SCRIPT_DIR}/run_e2d2_latency_read_methods.sh" "$@"
