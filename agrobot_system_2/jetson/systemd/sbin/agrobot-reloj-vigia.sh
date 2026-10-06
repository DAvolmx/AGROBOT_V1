#!/bin/bash
# Reinicia la pila ROS UNA vez cuando NTP sincroniza el reloj.
#
# Algo en el arranque de esta Jetson devuelve el reloj a enero de 1970 a los
# ~9 s (despues de que timesyncd lo restaura), y NTP lo corrige minutos mas
# tarde. Si ROS ya esta corriendo, ese salto de 56 anos deja a slam_toolbox
# atascado o lo mata con karto::Exception. Esperar a NTP antes de lanzar ROS
# no alcanza: el 2026-09-16 tardo 6 minutos, y en campo sin red no llega
# nunca. Sin red no hay salto y ROS funciona; con red, se reinicia una vez.
set -u

sincronizado() { [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; }

if sincronizado; then
    echo "[vigia-reloj] reloj ya sincronizado al arrancar — nada que hacer"
    exit 0
fi

echo "[vigia-reloj] reloj sin sincronizar ($(date)) — esperando a NTP"
until sincronizado; do sleep 5; done

echo "[vigia-reloj] NTP sincronizo ($(date)). Reiniciando agrobot-ros para que TF arranque con el reloj correcto."
# NO reiniciar a media subida (2026-10-02, dos veces): con NTP a los pocos
# segundos del arranque, el reinicio caía mientras los nodos se configuraban;
# slam_toolbox y controller_server morían (-6/-11), el relanzado quedaba
# colgado en "configurando" y Nav2 abortaba sin transform map. Se espera a
# que ROS lleve ACTIVO_MIN_S activo y se hace parada completa + arranque.
ACTIVO_MIN_S=60
activo_desde() {
    local ts
    ts=$(systemctl show agrobot-ros.service -p ActiveEnterTimestampMonotonic --value 2>/dev/null)
    [ -n "$ts" ] && [ "$ts" != "0" ] || { echo 0; return; }
    echo $(( ( $(cut -d. -f1 /proc/uptime) * 1000000 - ts ) / 1000000 ))
}
if systemctl is-active --quiet agrobot-ros.service; then
    while [ "$(activo_desde)" -lt "$ACTIVO_MIN_S" ]; do sleep 5; done
    systemctl stop agrobot-ros.service
    # Que no quede ningún nodo del lanzamiento anterior (respawn incluido).
    for _ in $(seq 1 20); do
        pgrep -f "slam_toolbox|nav2_|sllidar_node|ekf_node|rf2o" > /dev/null || break
        sleep 1
    done
    systemctl start agrobot-ros.service
fi
