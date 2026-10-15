#!/usr/bin/env bash
set -euo pipefail
RUN_DIR="${AGROBOT_GAZEBO_RUN_DIR:-$HOME/.agrobot/gazebo_sim_ros}"
for name in bridge gazebo; do
  pidfile="$RUN_DIR/$name.pid"
  [[ -s "$pidfile" ]] || continue
  pid="$(cat "$pidfile")"
  if kill -0 -- "-$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 -- "-$pid" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  fi
  rm -f "$pidfile"
  echo "$name detenido"
done
