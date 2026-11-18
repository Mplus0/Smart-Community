#!/usr/bin/env bash
# run.sh — start / stop / status helper for the infra workbench.
#
# Patterned after `vla-infra`'s console/run.sh: a single pidfile scoped
# to this repository so we never kill a infra owned by another
# checkout.
#
# Usage:
#   ./run.sh start          # background start; writes .infra.pid
#   ./run.sh stop           # SIGINT; escalates to SIGTERM/SIGKILL
#   ./run.sh restart
#   ./run.sh status         # prints pid + health JSON
#   ./run.sh foreground     # run in foreground (for roslaunch wrapping)

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "${HERE}/.." && pwd)"
PIDFILE="${HERE}/.infra.pid"
LOGFILE="${HERE}/.infra.log"
PORT="${INFRA_PORT:-8000}"
BIND="${INFRA_BIND:-0.0.0.0}"

cd "${ROBOT_WS}"

python_bin() {
  if command -v python3 >/dev/null 2>&1; then echo python3; else echo python; fi
}

is_ours() {
  # True when $1 is a pid whose cwd equals this repo root.
  local pid="$1"
  [[ -z "${pid}" ]] && return 1
  [[ ! -d "/proc/${pid}" ]] && return 1
  local pcwd
  pcwd="$(readlink "/proc/${pid}/cwd" 2>/dev/null || true)"
  [[ "${pcwd}" == "${ROBOT_WS}" ]]
}

cmd_start() {
  if [[ -f "${PIDFILE}" ]]; then
    local pid; pid="$(cat "${PIDFILE}")"
    if is_ours "${pid}"; then
      echo "infra already running (pid=${pid})"; return 0
    fi
    rm -f "${PIDFILE}"
  fi

  echo "starting infra on ${BIND}:${PORT} …"
  nohup "$(python_bin)" -m infra.server --host "${BIND}" --port "${PORT}" \
    >> "${LOGFILE}" 2>&1 &
  echo $! > "${PIDFILE}"
  sleep 1
  cmd_status || true
}

cmd_stop() {
  if [[ ! -f "${PIDFILE}" ]]; then echo "no pidfile; nothing to stop"; return 0; fi
  local pid; pid="$(cat "${PIDFILE}")"
  if ! is_ours "${pid}"; then
    echo "pid ${pid} is not our infra; removing stale pidfile"
    rm -f "${PIDFILE}"; return 0
  fi
  echo "stopping infra pid=${pid} …"
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    [[ -d "/proc/${pid}" ]] || break
    sleep 1
  done
  if [[ -d "/proc/${pid}" ]]; then
    echo "escalating to SIGTERM"
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in $(seq 1 5); do
      [[ -d "/proc/${pid}" ]] || break
      sleep 1
    done
  fi
  if [[ -d "/proc/${pid}" ]]; then
    echo "escalating to SIGKILL"
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${PIDFILE}"
}

cmd_status() {
  if [[ -f "${PIDFILE}" ]]; then
    local pid; pid="$(cat "${PIDFILE}")"
    if is_ours "${pid}"; then
      echo "infra running (pid=${pid})"
      curl -fsS "http://127.0.0.1:${PORT}/api/health" || true
      echo
      return 0
    fi
  fi
  echo "infra not running"
  return 1
}

cmd_foreground() {
  exec "$(python_bin)" -m infra.server --host "${BIND}" --port "${PORT}"
}

case "${1:-}" in
  start)      cmd_start ;;
  stop)       cmd_stop ;;
  restart)    cmd_stop || true; cmd_start ;;
  status)     cmd_status ;;
  foreground) cmd_foreground ;;
  *) echo "usage: $0 {start|stop|restart|status|foreground}"; exit 2 ;;
esac
