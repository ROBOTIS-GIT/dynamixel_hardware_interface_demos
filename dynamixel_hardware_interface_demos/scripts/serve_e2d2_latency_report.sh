#!/usr/bin/env bash
set -euo pipefail

PORT="${PORT:-8080}"
BIND="${BIND:-0.0.0.0}"
SESSION="${SESSION:-e2d2_report_http}"
RESTART="${RESTART:-1}"
FOREGROUND="${FOREGROUND:-0}"
SYNC_ONLY="${SYNC_ONLY:-0}"
CHANNELS="${CHANNELS:-1,2,3,4,6}"
CHANNELS_TAG="${CHANNELS_TAG:-${CHANNELS//,/-}}"
DURATION="${DURATION:-10}"

detect_workspace_root() {
  if [[ -d "/workspace/e2d2_latency_read_methods" ||
        -d "/workspace/e2d2_latency_read_methods_12byte" ]]; then
    echo "/workspace"
    return
  fi

  if [[ -d "/data/dynamixel_hardware_interface_demos/docker/workspace" ]]; then
    echo "/data/dynamixel_hardware_interface_demos/docker/workspace"
    return
  fi

  echo ""
}

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(detect_workspace_root)}"
if [[ -z "${WORKSPACE_ROOT}" || ! -d "${WORKSPACE_ROOT}" ]]; then
  echo "Cannot find E2D2 report workspace." >&2
  echo "Set WORKSPACE_ROOT, for example:" >&2
  echo "  WORKSPACE_ROOT=/data/dynamixel_hardware_interface_demos/docker/workspace $0" >&2
  exit 1
fi

if [[ -z "${PUBLIC_DIR:-}" ]]; then
  if [[ "${WORKSPACE_ROOT}" == /data/* || -d "/data" ]]; then
    PUBLIC_DIR="/data/e2d2_report_public"
  else
    PUBLIC_DIR="${WORKSPACE_ROOT}/e2d2_report_public"
  fi
fi

mkdir -p "${PUBLIC_DIR}"

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -f "${src}" ]]; then
    cp "${src}" "${dst}"
    echo "copied: ${dst}"
  else
    echo "missing: ${src}" >&2
  fi
}

REPORT4_BASE="e2d2_parallel_epoll_tcp_udp_summary_${CHANNELS_TAG}_${DURATION}s"
REPORT12_BASE="e2d2_parallel_epoll_tcp_udp_summary_${CHANNELS_TAG}_12byte_${DURATION}s"
REPORT4_DIR="${WORKSPACE_ROOT}/e2d2_latency_read_methods"
REPORT12_DIR="${WORKSPACE_ROOT}/e2d2_latency_read_methods_12byte"

echo "Sync E2D2 latency reports"
echo "  workspace=${WORKSPACE_ROOT}"
echo "  public=${PUBLIC_DIR}"
echo "  channels=${CHANNELS}"
echo "  duration=${DURATION}s"

copy_if_exists "${REPORT4_DIR}/${REPORT4_BASE}.html" "${PUBLIC_DIR}/index.html"
copy_if_exists "${REPORT4_DIR}/${REPORT4_BASE}.html" "${PUBLIC_DIR}/${REPORT4_BASE}.html"
copy_if_exists "${REPORT4_DIR}/${REPORT4_BASE}.svg" "${PUBLIC_DIR}/${REPORT4_BASE}.svg"

copy_if_exists "${REPORT12_DIR}/${REPORT12_BASE}.html" "${PUBLIC_DIR}/12byte.html"
copy_if_exists "${REPORT12_DIR}/${REPORT12_BASE}.html" "${PUBLIC_DIR}/${REPORT12_BASE}.html"
copy_if_exists "${REPORT12_DIR}/${REPORT12_BASE}.svg" "${PUBLIC_DIR}/${REPORT12_BASE}.svg"

for md in \
  "/data/e2d2_4byte_vs_12byte_latency_analysis_${CHANNELS_TAG}_${DURATION}s.md" \
  "${WORKSPACE_ROOT}/e2d2_4byte_vs_12byte_latency_analysis_${CHANNELS_TAG}_${DURATION}s.md"
do
  if [[ -f "${md}" ]]; then
    copy_if_exists "${md}" "${PUBLIC_DIR}/$(basename "${md}")"
    break
  fi
done

if [[ "${SYNC_ONLY}" == "1" ]]; then
  echo "sync only: ${PUBLIC_DIR}"
  exit 0
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required to serve the report." >&2
  exit 1
fi

print_urls() {
  echo
  echo "Open in Chrome:"
  echo "  http://127.0.0.1:${PORT}/"
  echo "  http://127.0.0.1:${PORT}/12byte.html"

  if command -v hostname >/dev/null 2>&1; then
    for ip in $(hostname -I 2>/dev/null || true); do
      case "${ip}" in
        127.*|"") ;;
        *)
          echo "  http://${ip}:${PORT}/"
          echo "  http://${ip}:${PORT}/12byte.html"
          ;;
      esac
    done
  fi

  if command -v ip >/dev/null 2>&1; then
    ip -4 addr show 2>/dev/null |
      awk '/inet / {sub(/\/.*/, "", $2); print $2}' |
      while read -r ip; do
        case "${ip}" in
          127.*|"") ;;
          *)
            echo "  http://${ip}:${PORT}/"
            echo "  http://${ip}:${PORT}/12byte.html"
            ;;
        esac
      done
  fi
}

if [[ "${FOREGROUND}" == "1" ]]; then
  print_urls
  cd "${PUBLIC_DIR}"
  exec python3 -m http.server "${PORT}" --bind "${BIND}"
fi

if command -v tmux >/dev/null 2>&1; then
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if [[ "${RESTART}" == "1" ]]; then
      tmux kill-session -t "${SESSION}"
    else
      echo "tmux session already exists: ${SESSION}"
      print_urls
      exit 0
    fi
  fi

  printf -v server_cmd 'cd %q && python3 -m http.server %q --bind %q' \
    "${PUBLIC_DIR}" "${PORT}" "${BIND}"
  tmux new-session -d -s "${SESSION}" "${server_cmd}"
  echo "server started in tmux session: ${SESSION}"
  echo "stop command: tmux kill-session -t ${SESSION}"
  print_urls
else
  echo "tmux is not available; running the HTTP server in the foreground."
  print_urls
  cd "${PUBLIC_DIR}"
  exec python3 -m http.server "${PORT}" --bind "${BIND}"
fi
