#!/usr/bin/env bash
set -euo pipefail

IP="${IP:-192.168.0.1}"
DXL_ID="${DXL_ID:-1}"
BAUDRATE="${BAUDRATE:-6000000}"
HZ="${HZ:-1000}"
CHANNEL="${CHANNEL:-1}"
CHANNELS="${CHANNELS:-1,2,3,4,6}"
SHORT_DURATION="${SHORT_DURATION:-10}"
LONG_DURATION="${LONG_DURATION:-60}"
OUT_DIR="${OUT_DIR:-/workspace/e2d2_latency}"
BUILD="${BUILD:-1}"
RT_PRIORITY="${RT_PRIORITY:-80}"
STRICT="${STRICT:-0}"
PRECHECK="${PRECHECK:-1}"
RUN_UDP="${RUN_UDP:-1}"
OPEN_RETRIES="${OPEN_RETRIES:-50}"
OPEN_RETRY_MS="${OPEN_RETRY_MS:-20}"
RESYNC_ON_MISS="${RESYNC_ON_MISS:-0}"
PLOT_TZ="${PLOT_TZ:-Asia/Seoul}"
COMPARE_TCP_UDP="${COMPARE_TCP_UDP:-1}"

WS="${WS:-/root/ros2_ws}"
PKG="dynamixel_hardware_interface_demos"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SCRIPT_DIR="${WS}/src/${PKG}/${PKG}/scripts"
if [[ -f "${PKG_SCRIPT_DIR}/plot_e2d2_latency.py" ]]; then
  PLOT_SCRIPT="${PLOT_SCRIPT:-${PKG_SCRIPT_DIR}/plot_e2d2_latency.py}"
else
  PLOT_SCRIPT="${PLOT_SCRIPT:-${SCRIPT_DIR}/plot_e2d2_latency.py}"
fi
if [[ -f "${PKG_SCRIPT_DIR}/plot_e2d2_latency_compare.py" ]]; then
  PLOT_COMPARE_SCRIPT="${PLOT_COMPARE_SCRIPT:-${PKG_SCRIPT_DIR}/plot_e2d2_latency_compare.py}"
else
  PLOT_COMPARE_SCRIPT="${PLOT_COMPARE_SCRIPT:-${SCRIPT_DIR}/plot_e2d2_latency_compare.py}"
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${WS}"

if [[ "${BUILD}" == "1" ]]; then
  colcon build --packages-select "${PKG}" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
set +u
source "${WS}/install/setup.bash"
set -u

mkdir -p "${OUT_DIR}"

expand_channels() {
  local spec="$1"
  local part
  echo "${spec}" | tr ',' '\n' | while read -r part; do
    if [[ "${part}" == *-* ]]; then
      local first="${part%-*}"
      local last="${part#*-}"
      seq "${first}" "${last}"
    elif [[ -n "${part}" ]]; then
      echo "${part}"
    fi
  done
}

probe_channels() {
  local transport="$1"
  echo "== precheck_${transport}_${CHANNELS} =="
  for channel in $(expand_channels "${CHANNELS}"); do
    set +e
    ros2 run "${PKG}" e2d2_read_latency_test \
      --mode single \
      --channel "${channel}" \
      --transport "${transport}" \
      --ip "${IP}" \
      --id "${DXL_ID}" \
      --baudrate "${BAUDRATE}" \
      --hz "${HZ}" \
      --duration 0.05 \
      --warmup 5 \
      --open-retries "${OPEN_RETRIES}" \
      --open-retry-ms "${OPEN_RETRY_MS}" \
      --read basic \
      --rt-priority "${RT_PRIORITY}" \
      --lock-memory
    local rc=$?
    set -e
    if [[ "${rc}" -eq 0 ]]; then
      echo "precheck ${transport} channel ${channel}: ok"
    else
      echo "precheck ${transport} channel ${channel}: failed rc=${rc}"
    fi
  done
}

run_case() {
  local name="$1"
  local title="$2"
  shift 2

  local csv_path="${OUT_DIR}/${name}.csv"
  local svg_path="${OUT_DIR}/${name}.svg"
  local resync_arg=()
  if [[ "${RESYNC_ON_MISS}" == "1" ]]; then
    resync_arg=(--resync-on-miss)
  fi

  echo "== ${name} =="
  rm -f "${csv_path}" "${svg_path}"
  set +e
  ros2 run "${PKG}" e2d2_read_latency_test \
    --ip "${IP}" \
    --id "${DXL_ID}" \
    --baudrate "${BAUDRATE}" \
    --hz "${HZ}" \
    --rt-priority "${RT_PRIORITY}" \
    --open-retries "${OPEN_RETRIES}" \
    --open-retry-ms "${OPEN_RETRY_MS}" \
    --lock-memory \
    "${resync_arg[@]}" \
    "$@" \
    --csv "${csv_path}"
  local rc=$?
  set -e

  if [[ -f "${csv_path}" ]] && [[ "$(wc -l < "${csv_path}")" -gt 1 ]]; then
    local generated_at
    generated_at="$(TZ="${PLOT_TZ}" date '+%Y-%m-%d %H:%M:%S %Z')"
    python3 "${PLOT_SCRIPT}" "${csv_path}" -o "${svg_path}" \
      --title "${title}" \
      --generated-at "${generated_at}"
  else
    echo "skip plot: ${csv_path} has no samples"
  fi

  if [[ "${rc}" -ne 0 ]]; then
    echo "case failed: ${name} rc=${rc}"
    if [[ "${STRICT}" == "1" ]]; then
      exit "${rc}"
    fi
  fi
}

has_samples() {
  local csv_path="$1"
  [[ -f "${csv_path}" ]] && [[ "$(wc -l < "${csv_path}")" -gt 1 ]]
}

plot_compare() {
  local name="$1"
  local title="$2"
  local first_label="$3"
  local first_csv="$4"
  local second_label="$5"
  local second_csv="$6"
  local svg_path="${OUT_DIR}/${name}.svg"

  if has_samples "${first_csv}" && has_samples "${second_csv}"; then
    local generated_at
    generated_at="$(TZ="${PLOT_TZ}" date '+%Y-%m-%d %H:%M:%S %Z')"
    python3 "${PLOT_COMPARE_SCRIPT}" "${first_csv}" "${second_csv}" \
      -o "${svg_path}" \
      --labels "${first_label},${second_label}" \
      --title "${title}" \
      --generated-at "${generated_at}"
  else
    echo "skip compare plot: missing samples for ${name}"
  fi
}

if [[ "${PRECHECK}" == "1" ]]; then
  probe_channels tcp
  if [[ "${RUN_UDP}" == "1" ]]; then
    probe_channels udp
  fi
fi

run_case \
  "basic_single_ch${CHANNEL}_tcp_${SHORT_DURATION}s" \
  "E2D2 Basic Read TCP, Channel ${CHANNEL}, ${SHORT_DURATION}s" \
  --mode single --channel "${CHANNEL}" --transport tcp --duration "${SHORT_DURATION}" --read basic

if [[ "${RUN_UDP}" == "1" ]]; then
  run_case \
    "basic_single_ch${CHANNEL}_udp_${SHORT_DURATION}s" \
    "E2D2 Basic Read UDP, Channel ${CHANNEL}, ${SHORT_DURATION}s" \
    --mode single --channel "${CHANNEL}" --transport udp --duration "${SHORT_DURATION}" --read basic

  if [[ "${COMPARE_TCP_UDP}" == "1" ]]; then
    plot_compare \
      "basic_single_ch${CHANNEL}_tcp_vs_udp_${SHORT_DURATION}s" \
      "E2D2 Basic Read TCP vs UDP, Channel ${CHANNEL}, ${SHORT_DURATION}s" \
      "TCP" "${OUT_DIR}/basic_single_ch${CHANNEL}_tcp_${SHORT_DURATION}s.csv" \
      "UDP" "${OUT_DIR}/basic_single_ch${CHANNEL}_udp_${SHORT_DURATION}s.csv"
  fi
fi

run_case \
  "basic_single_ch${CHANNEL}_tcp_long_${LONG_DURATION}s" \
  "E2D2 Basic Read TCP Long Run, Channel ${CHANNEL}, ${LONG_DURATION}s" \
  --mode single --channel "${CHANNEL}" --transport tcp --duration "${LONG_DURATION}" --read basic

run_case \
  "basic_parallel_threads_${CHANNELS}_tcp_${SHORT_DURATION}s" \
  "E2D2 Basic Read TCP, Parallel Channels ${CHANNELS}, ${SHORT_DURATION}s" \
  --mode parallel --channels "${CHANNELS}" --transport tcp --duration "${SHORT_DURATION}" --read basic

run_case \
  "basic_epoll_${CHANNELS}_tcp_${SHORT_DURATION}s" \
  "E2D2 Basic Read TCP, epoll Channels ${CHANNELS}, ${SHORT_DURATION}s" \
  --mode epoll --channels "${CHANNELS}" --transport tcp --duration "${SHORT_DURATION}" --read basic

echo "outputs: ${OUT_DIR}"
