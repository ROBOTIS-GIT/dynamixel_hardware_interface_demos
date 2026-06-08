#!/usr/bin/env bash
set -euo pipefail

CHANNELS="${CHANNELS:-1,2,3,4,6}"
CHANNELS_TAG="${CHANNELS_TAG:-${CHANNELS//,/-}}"
DURATION="${DURATION:-10}"
LEN="${LEN:-4}"
READ_METHODS="${READ_METHODS:-basic sync bulk fast-sync fast-bulk}"
OUT_ROOT="${OUT_ROOT:-/workspace/e2d2_latency_read_methods}"
OUTPUT="${OUTPUT:-${OUT_ROOT}/e2d2_parallel_epoll_tcp_udp_summary_${CHANNELS_TAG}_${DURATION}s.svg}"
PLOT_TZ="${PLOT_TZ:-Asia/Seoul}"
X_LIMIT_US="${X_LIMIT_US:-0}"

WS="${WS:-/root/ros2_ws}"
PKG="dynamixel_hardware_interface_demos"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SCRIPT_DIR="${WS}/src/${PKG}/${PKG}/scripts"
if [[ -f "${PKG_SCRIPT_DIR}/plot_e2d2_latency_compare.py" ]]; then
  PLOT_COMPARE_SCRIPT="${PLOT_COMPARE_SCRIPT:-${PKG_SCRIPT_DIR}/plot_e2d2_latency_compare.py}"
else
  PLOT_COMPARE_SCRIPT="${PLOT_COMPARE_SCRIPT:-${SCRIPT_DIR}/plot_e2d2_latency_compare.py}"
fi

csvs=()
labels=()

has_samples() {
  local csv_path="$1"
  [[ -f "${csv_path}" ]] && [[ "$(wc -l < "${csv_path}")" -gt 1 ]]
}

add_series() {
  local label="$1"
  local csv_path="$2"
  if has_samples "${csv_path}"; then
    labels+=("${label}")
    csvs+=("${csv_path}")
  else
    echo "skip missing/empty: ${label} ${csv_path}"
  fi
}

join_by_comma() {
  local IFS=","
  echo "$*"
}

for transport in tcp udp; do
  for read_method in ${READ_METHODS}; do
    add_series \
      "${transport^^} parallel ${read_method}" \
      "${OUT_ROOT}/${transport}/parallel_${read_method}_${CHANNELS_TAG}_${transport}_${DURATION}s.csv"
  done
done

for transport in tcp udp; do
  add_series \
    "${transport^^} epoll basic" \
    "${OUT_ROOT}/${transport}/epoll_basic_${CHANNELS_TAG}_${transport}_${DURATION}s.csv"
  add_series \
    "${transport^^} epoll-sdk basic" \
    "${OUT_ROOT}/${transport}/epoll-sdk_basic_${CHANNELS_TAG}_${transport}_${DURATION}s.csv"
done

if [[ "${#csvs[@]}" -eq 0 ]]; then
  echo "no CSV samples found under ${OUT_ROOT}" >&2
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT}")"
generated_at="$(TZ="${PLOT_TZ}" date '+%Y-%m-%d %H:%M:%S %Z')"

python3 "${PLOT_COMPARE_SCRIPT}" "${csvs[@]}" \
  -o "${OUTPUT}" \
  --labels "$(join_by_comma "${labels[@]}")" \
  --title "E2D2 TCP/UDP Parallel and epoll Read Latency, Channels ${CHANNELS}, ${LEN} bytes, ${DURATION}s" \
  --generated-at "${generated_at}" \
  --x-limit-us "${X_LIMIT_US}"

echo "summary: ${OUTPUT}"
