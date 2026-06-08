#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LEN=12
export OUT_ROOT="${OUT_ROOT:-/workspace/e2d2_latency_read_methods_12byte}"
exec "${SCRIPT_DIR}/run_e2d2_latency_read_methods_udp.sh" "$@"
