#!/usr/bin/env bash
# Levanta el nodo del RPLIDAR C1 y publica /scan, que es de donde el
# dashboard saca los puntos del radar.
#
#   ./lanzar_lidar.sh
#
# Sin esto el tab del LIDAR se ve vacío aunque el sensor esté enchufado: el
# backend no habla con el puerto serie, solo escucha el topic /scan.
set -e
cd "$(dirname "$0")"
DISTRO=$(ls /opt/ros 2>/dev/null | while read -r d; do
           [ -f "/opt/ros/$d/setup.bash" ] && echo "$d"; done | head -1)
[ -n "$DISTRO" ] || { echo "ROS no está instalado"; exit 1; }
# shellcheck disable=SC1090
source "/opt/ros/$DISTRO/setup.bash"
source "$(cd ../../ros2_ws && pwd)/install/setup.bash"

# UART del conector de 40 pines desde el 2026-09-29 (antes /dev/ttyUSB0).
PUERTO=${1:-/dev/ttyTHS1}
[ -c "$PUERTO" ] || { echo "No existe $PUERTO — ¿está enchufado el LIDAR?"; exit 1; }

# El puerto es root:dialout 660. Caso frecuente y confuso: el usuario YA fue
# agregado al grupo dialout (`groups <user>` lo muestra) pero la sesión de
# login es anterior al cambio, así que el proceso NO lo tiene (`id` no lo
# lista) y el nodo muere con "code: 80008004", que no dice nada de permisos.
# Si es ese el caso, se re-lanza con `sg dialout`, que usa la membresía que
# ya existe — no modifica nada del sistema. El arreglo definitivo es cerrar
# sesión y volver a entrar (o reiniciar).
if [ ! -r "$PUERTO" ] && [ -z "$AGROBOT_SG" ]; then
  if id -nG "$USER" | grep -qw dialout; then
    echo "[lidar] sin permiso en $PUERTO en esta sesión, pero sí en el grupo:"
    echo "        re-lanzando con 'sg dialout'. Para arreglarlo de raíz,"
    echo "        cerrá sesión y volvé a entrar (o reiniciá)."
    export AGROBOT_SG=1
    exec sg dialout -c "$(printf '%q ' "$0" "$@")"
  fi
  echo "⚠️  Sin permiso de lectura en $PUERTO y no estás en el grupo dialout."
  echo "    sudo usermod -aG dialout $USER   (y después cerrar sesión)"
  exit 1
fi

# 460800 es el baudrate REAL del C1, no los 230400 del C1 viejo/A-series.
echo "[lidar] $PUERTO a 460800 baudios"
exec ros2 launch sllidar_ros2 sllidar_c1_launch.py \
     serial_port:="$PUERTO" serial_baudrate:=460800
