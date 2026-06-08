#!/usr/bin/env bash
set -euo pipefail

IP="${IP:-192.168.0.1}"
DXL_ID="${DXL_ID:-1}"
BAUDRATE="${BAUDRATE:-6000000}"
HZ="${HZ:-1000}"
CHANNELS="${CHANNELS:-1,2,3,4,6}"
CHANNELS_TAG="${CHANNELS_TAG:-${CHANNELS//,/-}}"
DURATION="${DURATION:-10}"
ADDR="${ADDR:-132}"
LEN="${LEN:-4}"
TRANSPORT="${TRANSPORT:-tcp}"
READ_METHODS="${READ_METHODS:-basic sync bulk fast-sync fast-bulk}"
RUN_PARALLEL="${RUN_PARALLEL:-1}"
RUN_EPOLL="${RUN_EPOLL:-1}"
RUN_EPOLL_SDK="${RUN_EPOLL_SDK:-0}"
OUT_ROOT="${OUT_ROOT:-/workspace/e2d2_latency_read_methods}"
OUT_DIR="${OUT_DIR:-${OUT_ROOT}/${TRANSPORT}}"
BUILD="${BUILD:-1}"
RT_PRIORITY="${RT_PRIORITY:-80}"
STRICT="${STRICT:-0}"
OPEN_RETRIES="${OPEN_RETRIES:-50}"
OPEN_RETRY_MS="${OPEN_RETRY_MS:-20}"
WARMUP="${WARMUP:-50}"
RESYNC_ON_MISS="${RESYNC_ON_MISS:-0}"
PLOT_TZ="${PLOT_TZ:-Asia/Seoul}"

WS="${WS:-/root/ros2_ws}"
PKG="dynamixel_hardware_interface_demos"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SCRIPT_DIR="${WS}/src/${PKG}/${PKG}/scripts"
if [[ -f "${PKG_SCRIPT_DIR}/plot_e2d2_latency.py" ]]; then
  PLOT_SCRIPT="${PLOT_SCRIPT:-${PKG_SCRIPT_DIR}/plot_e2d2_latency.py}"
else
  PLOT_SCRIPT="${PLOT_SCRIPT:-${SCRIPT_DIR}/plot_e2d2_latency.py}"
fi

if [[ "${TRANSPORT}" != "tcp" && "${TRANSPORT}" != "udp" ]]; then
  echo "TRANSPORT must be tcp or udp, got: ${TRANSPORT}" >&2
  exit 2
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

run_case() {
  local mode="$1"
  local read_method="$2"
  local title="$3"
  local name="${mode}_${read_method}_${CHANNELS_TAG}_${TRANSPORT}_${DURATION}s"
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
    --transport "${TRANSPORT}" \
    --mode "${mode}" \
    --channels "${CHANNELS}" \
    --ip "${IP}" \
    --id "${DXL_ID}" \
    --baudrate "${BAUDRATE}" \
    --hz "${HZ}" \
    --duration "${DURATION}" \
    --read "${read_method}" \
    --addr "${ADDR}" \
    --len "${LEN}" \
    --warmup "${WARMUP}" \
    --rt-priority "${RT_PRIORITY}" \
    --open-retries "${OPEN_RETRIES}" \
    --open-retry-ms "${OPEN_RETRY_MS}" \
    --lock-memory \
    "${resync_arg[@]}" \
    --csv "${csv_path}"
  local rc=$?
  set -e

  if [[ -f "${csv_path}" ]] && [[ "$(wc -l < "${csv_path}")" -gt 1 ]]; then
    local generated_at
    generated_at="$(TZ="${PLOT_TZ}" date '+%Y-%m-%d %H:%M:%S %Z')"
    python3 "${PLOT_SCRIPT}" "${csv_path}" \
      -o "${svg_path}" \
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

echo "E2D2 latency read-method run"
echo "  transport=${TRANSPORT}"
echo "  channels=${CHANNELS}"
echo "  read_methods=${READ_METHODS}"
echo "  addr=${ADDR} len=${LEN} hz=${HZ} duration=${DURATION}s"
echo "  out_dir=${OUT_DIR}"

if [[ "${RUN_PARALLEL}" == "1" ]]; then
  for read_method in ${READ_METHODS}; do
    run_case \
      "parallel" \
      "${read_method}" \
      "E2D2 ${read_method} Read ${TRANSPORT^^}, Parallel Channels ${CHANNELS}, ${LEN} bytes, ${DURATION}s"
  done
fi

if [[ "${RUN_EPOLL}" == "1" ]]; then
  run_case \
    "epoll" \
    "basic" \
    "E2D2 basic Read ${TRANSPORT^^}, epoll Channels ${CHANNELS}, ${LEN} bytes, ${DURATION}s"
fi

if [[ "${RUN_EPOLL_SDK}" == "1" ]]; then
  run_case \
    "epoll-sdk" \
    "basic" \
    "E2D2 basic Read ${TRANSPORT^^}, SDK epoll Channels ${CHANNELS}, ${LEN} bytes, ${DURATION}s"
fi

echo "outputs: ${OUT_DIR}"
