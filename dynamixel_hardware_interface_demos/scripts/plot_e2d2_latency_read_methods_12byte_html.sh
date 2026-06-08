#!/usr/bin/env bash
set -euo pipefail

CHANNELS="${CHANNELS:-1,2,3,4,6}"
CHANNELS_TAG="${CHANNELS_TAG:-${CHANNELS//,/-}}"
DURATION="${DURATION:-10}"
LEN=12
OUT_ROOT="${OUT_ROOT:-/workspace/e2d2_latency_read_methods_12byte}"
SVG_PATH="${SVG_PATH:-${OUT_ROOT}/e2d2_parallel_epoll_tcp_udp_summary_${CHANNELS_TAG}_12byte_${DURATION}s.svg}"
OUTPUT="${OUTPUT:-${OUT_ROOT}/e2d2_parallel_epoll_tcp_udp_summary_${CHANNELS_TAG}_12byte_${DURATION}s.html}"

WS="${WS:-/root/ros2_ws}"
PKG="dynamixel_hardware_interface_demos"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SCRIPT_DIR="${WS}/src/${PKG}/${PKG}/scripts"
if [[ -f "${PKG_SCRIPT_DIR}/plot_e2d2_latency_read_methods_html.py" ]]; then
  HTML_SCRIPT="${HTML_SCRIPT:-${PKG_SCRIPT_DIR}/plot_e2d2_latency_read_methods_html.py}"
else
  HTML_SCRIPT="${HTML_SCRIPT:-${SCRIPT_DIR}/plot_e2d2_latency_read_methods_html.py}"
fi

mkdir -p "$(dirname "${OUTPUT}")"

python3 "${HTML_SCRIPT}" \
  --out-root "${OUT_ROOT}" \
  --channels "${CHANNELS}" \
  --duration "${DURATION}" \
  --length "${LEN}" \
  --svg "${SVG_PATH}" \
  --output "${OUTPUT}" \
  --title "E2D2 TCP/UDP Parallel and epoll Read Latency, Channels ${CHANNELS}, ${LEN} bytes, ${DURATION}s"

echo "html: ${OUTPUT}"
