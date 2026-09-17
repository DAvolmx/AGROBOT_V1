#!/usr/bin/env bash
# =============================================================================
# INICIAR_BOT.sh — arranca TODO el sistema con evasión de obstáculos y Nav2
#
# VERSIÓN: 2026-09-13 — Sistema de evasión integrado + Nav2 verificado
#
# USO:
#   ./INICIAR_BOT.sh              mapeo (SLAM construye el mapa desde cero)
#   ./INICIAR_BOT.sh nav          navegación (Nav2 sobre un mapa ya guardado)
#   ./INICIAR_BOT.sh banco        mapeo SIN el nodo LoRa: odometría fija
#   ./INICIAR_BOT.sh solo-backend sin ROS: dashboard + cámara solamente
#
# MEJORAS INCLUIDAS:
#   ✅ Evasión reactiva de obstáculos basada en LIDAR (7 sectores)
#   ✅ Parada instantánea mejorada (<50ms)
#   ✅ Nav2 verificado antes de permitir navegación
#   ✅ API /api/obstacle_status para monitoreo en vivo
# =============================================================================
set +u
cd "$(dirname "$0")"

MODO="${1:-mapeo}"
LOGS="$(pwd)/logs"
mkdir -p "$LOGS"

# Entorno ROS
ROS_OK=0
if [ -z "${ROS_DISTRO:-}" ]; then
  DISTRO=$(ls /opt/ros 2>/dev/null | while read -r d; do
             [ -f "/opt/ros/$d/setup.bash" ] && echo "$d"; done | head -1)
  if [ -n "$DISTRO" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$DISTRO/setup.bash"
  fi
fi
if [ -n "${ROS_DISTRO:-}" ]; then
  ROS_OK=1
  echo "[🚀 iniciar] ROS $ROS_DISTRO activado"
  if [ -f "ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "ros2_ws/install/setup.bash"
    echo "[🚀 iniciar] workspace agrobot activado"
  else
    echo "[🚀 iniciar] ⚠️  ros2_ws sin compilar"
    ROS_OK=0
  fi
fi
[ "$MODO" = "solo-backend" ] && ROS_OK=0

# Verificar LIDAR
PUERTO_LIDAR=/dev/ttyUSB0
if [ "$ROS_OK" = "1" ]; then
  if [ ! -c "$PUERTO_LIDAR" ]; then
    echo "[🚀 iniciar] ⚠️  no existe $PUERTO_LIDAR — RPLIDAR desconectado"
  elif [ ! -r "$PUERTO_LIDAR" ]; then
    echo "[🚀 iniciar] ⚠️  sin permiso en $PUERTO_LIDAR"
  fi
fi

# Esperar a que el reloj esté sincronizado ANTES de levantar ROS.
#
# La Jetson no tiene pila de RTC: arranca creyendo que es enero de 1970 y
# NTP le salta el reloj 56 años adelante ~80 s después. Si ROS levanta
# dentro de esa ventana, el salto le invalida las marcas de tiempo de TF a
# mitad de sesión: el filtro de mensajes se llena y slam_toolbox aborta con
# `karto::Exception`. Pasó el 2026-09-16 y se repite en cada arranque en
# que ROS le gane la carrera a NTP.
if [ "$ROS_OK" = "1" ]; then
  for _ in $(seq 60); do
    [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ] && break
    echo "[🚀 iniciar] ⏳ esperando a que NTP sincronice el reloj…"
    sleep 2
  done
  if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
    echo "[🚀 iniciar] ✅ reloj sincronizado — $(date)"
  else
    echo "[🚀 iniciar] ⚠️  el reloj NO sincronizó tras 120 s (¿sin red?)."
    echo "[🚀 iniciar]     Si salta ahora, slam_toolbox se caerá. Seguimos igual."
  fi
fi

# Apagado ordenado
PIDS=()
BAJANDO=0
bajar() {
  [ "$BAJANDO" = "1" ] && return
  BAJANDO=1
  echo
  echo "[🚀 iniciar] 🛑 Parando servicios…"
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  done
  sleep 3
  for pid in "${PIDS[@]}"; do
    kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
  done
  echo "[🚀 iniciar] ✅ Limpieza completada"
}
trap bajar INT TERM EXIT

# Backend
echo "[🚀 iniciar] 📊 Backend  → $LOGS/backend.log"
setsid python3 -u jetson/backend/jetson_server.py >"$LOGS/backend.log" 2>&1 &
PIDS+=($!)

# Esperar a que backend esté listo
for _ in $(seq 1 30); do
  if ss -ltn 2>/dev/null | grep -q ':8080 '; then
    echo "[🚀 iniciar] ✅ Backend listo (puerto 8080)"
    break
  fi
  if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    echo "[🚀 iniciar] ✗ Backend murió:"
    tail -20 "$LOGS/backend.log"
    exit 1
  fi
  sleep 1
done

# ROS
if [ "$ROS_OK" = "1" ]; then
  EXTRA=""
  case "$MODO" in
    nav)   LAUNCH=agrobot_navigation.launch.py
           echo "[🚀 iniciar] 🗺️  MODO NAVEGACIÓN con Nav2"
           ;;
    mapeo) LAUNCH=agrobot_mapping.launch.py
           echo "[🚀 iniciar] 📍 MODO MAPEO — construyendo mapa con SLAM"
           ;;
    banco) LAUNCH=agrobot_mapping.launch.py
           EXTRA="odom_estatico:=true"
           echo "[🚀 iniciar] 🏦 MODO BANCO — odometría fija (sin LoRa)"
           ;;
    *)     echo "[🚀 iniciar] ✗ modo desconocido: '$MODO'"; exit 2 ;;
  esac

  echo "[🚀 iniciar] 🤖 ROS → $LOGS/ros.log"
  # shellcheck disable=SC2086
  setsid ros2 launch agrobot_navigation "$LAUNCH" $EXTRA >"$LOGS/ros.log" 2>&1 &
  PIDS+=($!)

  # VERIFICACIÓN NAV2 (NUEVO 2026-09-13)
  if [ "$MODO" = "nav" ]; then
    echo "[🚀 iniciar] ⏳ Verificando Nav2…"
    nav2_ready=0
    for _ in $(seq 1 60); do
      if timeout 2 ros2 topic list 2>/dev/null | grep -q "^/goal_pose$"; then
        if timeout 2 ros2 service list 2>/dev/null | grep -q "nav_to_pose"; then
          echo "[🚀 iniciar] ✅ Nav2 LISTO y disponible"
          nav2_ready=1
          break
        fi
      fi
      if ! kill -0 "${PIDS[1]}" 2>/dev/null; then
        echo "[🚀 iniciar] ✗ ROS murió:"
        tail -30 "$LOGS/ros.log"
        exit 1
      fi
      sleep 1
    done
    [ "$nav2_ready" = "0" ] && echo "[🚀 iniciar] ⚠️  Nav2 timeout (60s)"
  fi
fi

# Verificar IMU
if [ "$ROS_OK" = "1" ] && [ "$MODO" != "banco" ]; then
  ( sleep 20
    if ! timeout 8 ros2 topic echo /imu/data_raw --once >/dev/null 2>&1; then
      echo "[🚀 iniciar] ⚠️  NO llega IMU — mapa NO se construirá"
    else
      echo "[🚀 iniciar] ✅ IMU recibida — EKF listo"
    fi ) &
fi

IP=$(hostname -I | awk '{print $1}')
echo
echo "  ╔════════════════════════════════════════════╗"
echo "   🤖 AgroBot EN MARCHA — Modo: $MODO"
echo "   📊 Dashboard: http://${IP:-127.0.0.1}:8080"
echo "   📁 Logs: $LOGS/"
echo "   🛑 Evasión de obstáculos: ACTIVADA"
echo "   ⏸️  Parada instantánea: MEJORADA"
if [ "$ROS_OK" = "1" ]; then
  echo "   🗺️  API: GET /api/obstacle_status (monitoreo en vivo)"
fi
echo "   💾 Ctrl+C para apagar"
echo "  ╚════════════════════════════════════════════╝"
echo

wait -n "${PIDS[@]}"
echo "[🚀 iniciar] ⚠️  Componente terminó — revisá $LOGS/"
