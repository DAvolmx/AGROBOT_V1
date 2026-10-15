#!/usr/bin/env bash
set -eo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$HERE/../.." && pwd)"
RUN_DIR="${AGROBOT_GAZEBO_RUN_DIR:-$HOME/.agrobot/gazebo_sim_ros}"
DOMAIN_ID="${AGROBOT_SIM_ROS_DOMAIN_ID:-42}"
WORLD="${AGROBOT_GAZEBO_WORLD:-$HERE/generados/huerta_mixta.sdf}"
BRIDGE="$HERE/bridge.yaml"

if [[ "$DOMAIN_ID" == "0" ]]; then
  echo "ERROR: ROS_DOMAIN_ID=0 está reservado al robot real. Use 42 para simulación." >&2
  exit 2
fi
if [[ ! -f "$WORLD" || ! -f "$BRIDGE" ]]; then
  echo "ERROR: falta el mundo o bridge.yaml" >&2
  exit 2
fi
mkdir -p "$RUN_DIR"
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export GZ_SIM_RESOURCE_PATH="$HERE${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if [[ "$WORLD" == "$HERE/generados/huerta_mixta.sdf" ]]; then
  python3 "$HERE/generar_escenario.py" --modo mixto --semilla 20261012 --salida "$WORLD"
fi

start_one() {
  local name="$1"; shift
  local pidfile="$RUN_DIR/$name.pid"
  if [[ -s "$pidfile" ]] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
    echo "$name ya está activo (PID $(cat "$pidfile"))"
    return
  fi
  rm -f "$pidfile"
  nohup setsid "$@" >"$RUN_DIR/$name.log" 2>&1 </dev/null &
  echo $! >"$pidfile"
  echo "$name iniciado (PID $(cat "$pidfile"))"
}

start_one gazebo gz sim --headless-rendering -s -r "$WORLD"
sleep 4
if ! kill -0 "$(cat "$RUN_DIR/gazebo.pid")" 2>/dev/null; then
  echo "ERROR: Gazebo terminó al arrancar; revisar $RUN_DIR/gazebo.log" >&2
  exit 1
fi
start_one bridge ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$BRIDGE"
sleep 3
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (simulación aislada del robot real)"
echo "Run dir: $RUN_DIR"
echo "Verificar: source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 topic list"
echo "Controlador de prueba: ROS_DOMAIN_ID=$ROS_DOMAIN_ID python3 $HERE/seguir_sendero.py --duration 20 --salida $RUN_DIR/metrics.json"
echo "Detener: $HERE/detener_ros_gazebo.sh"
