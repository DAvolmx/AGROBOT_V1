#!/usr/bin/env bash
# =============================================================================
# verificar_slam.sh — diagnóstico del SLAM de AgroBot.
#
# El SLAM puede fallar de varias formas que dan EXACTAMENTE el mismo síntoma
# desde afuera ("el mapa no aparece") y ninguna imprime un error claro:
#
#   1. El LIDAR no publica /scan (cable, permisos del puerto, puerto cambiado).
#   2. Falta el TF base_link->laser: slam_toolbox descarta todos los scans
#      en silencio.
#   3. Falta el eslabón odom->base_link porque el EKF nunca recibió una
#      medición (jetson_server.py apagado, o sin telemetría del MPU6050).
#   4. slam_toolbox está vivo pero en estado `unconfigured`: es un lifecycle
#      node y sin activarlo ni siquiera se suscribe a /scan.
#
# Este script distingue las cuatro. Uso, con el launch YA corriendo en otra
# terminal:
#
#   ros2 launch agrobot_navigation agrobot_mapping.launch.py
#   ./verificar_slam.sh
#
# Para probar sin robot ni backend (solo LIDAR sobre la mesa):
#   ros2 launch agrobot_navigation agrobot_mapping.launch.py odom_estatico:=true
# =============================================================================
# Sin `pipefail` a propósito: `grep -q` corta la tubería en cuanto encuentra
# lo que busca, y eso le manda SIGPIPE al `ros2 ...` de la izquierda. Con
# pipefail ese SIGPIPE se leería como "el comando falló" y el chequeo daría
# rojo justo cuando el dato SÍ estaba.
set -u

ok()   { echo -e "  \033[32m✓\033[0m $1"; }
bad()  { echo -e "  \033[31m✗\033[0m $1"; FALLOS=$((FALLOS+1)); }
info() { echo -e "  \033[36mi\033[0m $1"; }
FALLOS=0

echo
echo "=== Verificación del SLAM de AgroBot ==="
echo

echo "[1/6] Entorno ROS 2"
if [ -z "${ROS_DISTRO:-}" ]; then
  bad "ROS no está activado. Ejecutá: source /opt/ros/jazzy/setup.bash && source install/setup.bash"
  exit 1
fi
ok "ROS_DISTRO=$ROS_DISTRO"
python3 -c "import rclpy" 2>/dev/null && ok "rclpy importable" || bad "rclpy no importable"

echo
echo "[2/6] Puerto serie del LIDAR"
if [ -e /dev/ttyUSB0 ]; then
  ok "/dev/ttyUSB0 existe"
  id -nG | grep -qw dialout && ok "el usuario está en el grupo 'dialout'" \
    || bad "el usuario NO está en 'dialout' — 'sudo usermod -aG dialout \$USER' y reiniciar sesión"
else
  bad "/dev/ttyUSB0 no existe — revisá el cable USB del RPLIDAR o ajustá serial_port en el launch"
fi

echo
echo "[3/6] Nodos vivos"
NODOS=$(ros2 node list 2>/dev/null)
for n in sllidar_node ekf_filter_node slam_toolbox; do
  echo "$NODOS" | grep -q "/$n" && ok "$n" || bad "$n no está corriendo — ¿lanzaste el launch?"
done

echo
echo "[4/6] Ciclo de vida de slam_toolbox"
ESTADO=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)
case "$ESTADO" in
  active*) ok "slam_toolbox está 'active'" ;;
  "")      bad "no se pudo consultar el estado (nodo caído)" ;;
  *)       bad "slam_toolbox está '$ESTADO', no 'active' — el lifecycle_manager no lo activó.
       Activación manual: ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate" ;;
esac

echo
echo "[5/6] Árbol TF"
for par in "base_link laser" "base_link imu_link" "odom base_link" "map base_link"; do
  set -- $par
  SALIDA=$(timeout 5 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1)
  if echo "$SALIDA" | grep -q "Translation"; then
    ok "TF $1 -> $2"
  else
    case "$1 $2" in
      "odom base_link") bad "falta TF odom -> base_link — el EKF no recibió ninguna medición.
       Arrancá jetson_server.py (publica /imu/data_raw), o probá con odom_estatico:=true" ;;
      "map base_link")  bad "falta TF map -> base_link — slam_toolbox no está procesando scans" ;;
      *)                bad "falta TF $1 -> $2 — revisá los static_transform_publisher del launch" ;;
    esac
  fi
done

echo
echo "[6/6] Tópicos de datos"
HZ=$(timeout 8 ros2 topic hz /scan 2>&1)
if echo "$HZ" | grep -q "average rate"; then
  ok "/scan publicando (el LIDAR gira y entrega datos)"
else
  bad "/scan sin datos"
fi
SUBS=$(ros2 topic info /scan 2>/dev/null | grep "Subscription count" | grep -o '[0-9]*')
[ "${SUBS:-0}" -ge 1 ] && ok "/scan tiene $SUBS suscriptor(es)" \
  || bad "/scan no tiene suscriptores — slam_toolbox no está escuchando (ver paso 4)"

if timeout 15 ros2 topic echo /map --once --field info >/dev/null 2>&1; then
  ok "/map publicado — el SLAM está construyendo el mapa"
else
  bad "/map no se publica todavía (puede tardar unos segundos tras activar)"
fi

echo
if [ "$FALLOS" -eq 0 ]; then
  echo -e "\033[32m=== SLAM OK — $FALLOS fallos ===\033[0m"
  echo "Guardar el mapa:  ros2 run nav2_map_server map_saver_cli -f src/agrobot_navigation/maps/agrobot_map"
else
  echo -e "\033[31m=== $FALLOS comprobación(es) fallaron — ver arriba ===\033[0m"
fi
echo
exit "$FALLOS"
