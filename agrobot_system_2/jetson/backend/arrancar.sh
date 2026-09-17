#!/usr/bin/env bash
# Arranca el backend de la Jetson CON ROS 2 activado.
#
#   ./arrancar.sh
#
# Existe por un motivo concreto: lanzar `python3 jetson_server.py` a secas
# funciona —el backend arranca igual— pero rclpy no es importable y todo lo
# de ROS queda apagado en silencio. El dashboard mostraba "ROS no
# disponible" y era indistinguible de no tener ROS instalado.
#
# Esto NO levanta los nodos de ROS (LIDAR, slam_toolbox, Nav2): esos van
# aparte, ver `lanzar_lidar.sh` y el README §Navegación.
set -e
cd "$(dirname "$0")"

if [ -z "$ROS_DISTRO" ]; then
  # Se elige el distro que realmente esté instalado en vez de fijar uno:
  # cambiar de versión de ROS no debería obligar a editar este script.
  DISTRO=$(ls /opt/ros 2>/dev/null | while read -r d; do
             [ -f "/opt/ros/$d/setup.bash" ] && echo "$d"; done | head -1)
  if [ -n "$DISTRO" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/$DISTRO/setup.bash"
    echo "[arrancar] ROS $DISTRO activado"
  else
    echo "[arrancar] ⚠️  ROS no está instalado: el backend corre igual, con"
    echo "           el SLAM casero de slam.py. Nav2 y el mapa no estarán."
  fi
fi

# El workspace propio (sllidar_ros2 + agrobot_navigation), si está compilado
WS="$(cd ../../ros2_ws && pwd)"
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/install/setup.bash"
  echo "[arrancar] workspace agrobot activado"
fi

echo "[arrancar] dashboard en http://$(hostname -I | awk '{print $1}'):8080"
exec python3 jetson_server.py
