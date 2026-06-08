#!/usr/bin/env bash
set -euo pipefail

CHANNELS="${CHANNELS:-1,2,3,4,6}"
CHANNELS_TAG="${CHANNELS_TAG:-${CHANNELS//,/-}}"
DURATION="${DURATION:-10}"
export LEN=12
export OUT_ROOT="${OUT_ROOT:-/workspace/e2d2_latency_read_methods_12byte}"
export OUTPUT="${OUTPUT:-${OUT_ROOT}/e2d2_parallel_epoll_tcp_udp_summary_${CHANNELS_TAG}_12byte_${DURATION}s.svg}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/plot_e2d2_latency_read_methods_summary.sh" "$@"
