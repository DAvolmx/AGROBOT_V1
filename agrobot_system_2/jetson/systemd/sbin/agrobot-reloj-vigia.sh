#!/bin/bash
# Reinicia la pila ROS UNA vez cuando NTP sincroniza el reloj — pero con el robot EN REPOSO.
#
# Algo en el arranque de esta Jetson devuelve el reloj a enero de 1970 a los
# ~9 s (despues de que timesyncd lo restaura), y NTP lo corrige minutos mas
# tarde. Si ROS ya esta corriendo, ese salto deja a slam_toolbox atascado o lo
# mata con karto::Exception. Esperar a NTP antes de lanzar ROS no alcanza: el
# 2026-09-16 tardo 6 minutos, y en campo sin red no llega nunca. Sin red no hay
# salto y ROS funciona; con red, se reinicia una vez.
#
# 2026-10-27: el reinicio caia en PLENA MISION (NTP llego 20 min despues del
# arranque, +711 s): ROS se reinicio con el robot trabajando, el mapa se borro y
# la pose del SLAM salto 30 y 24 m. Ahora, antes de reiniciar ROS se espera a
# que el backend diga que el robot esta en reposo (GET /api/ocupado: sin
# autonomia, mision, pulsos ni movimiento reciente) durante IDLE_S seguidos.
set -u

VIGIA_URL="${VIGIA_URL:-http://127.0.0.1:8080/api/ocupado}"
IDLE_S="${IDLE_S:-30}"               # segundos seguidos en reposo antes de reiniciar ROS
SONDEO_S="${SONDEO_S:-5}"
ESPERA_MAX_S="${ESPERA_MAX_S:-3600}" # tope de espera: pasada una hora se reinicia igual (una pose corrupta es peor)
ACTIVO_MIN_S="${ACTIVO_MIN_S:-60}"

sincronizado() { [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; }

# 0 = ocupado; 1 = en reposo. Si el backend no responde (caido o arrancando), cuenta como reposo.
ocupado() {
    local r
    r=$(curl -s -m 3 "$VIGIA_URL" 2>/dev/null) || return 1
    case "$r" in *'"ocupado": true'*|*'"ocupado":true'*) return 0 ;; *) return 1 ;; esac
}

# aritmetica decimal (SONDEO_S puede ser 0.2 en las pruebas)
mas() { awk -v a="$1" -v b="$2" 'BEGIN{printf "%.3f", a+b}'; }
menor() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a<b)}'; }

esperar_reposo() {
    local quieto=0 espera=0 ult_aviso=-100
    while menor "$quieto" "$IDLE_S" && menor "$espera" "$ESPERA_MAX_S"; do
        if ocupado; then
            quieto=0
            if ! menor "$(awk -v e="$espera" -v u="$ult_aviso" 'BEGIN{print e-u}')" 60; then
                echo "[vigia-reloj] el robot esta ocupado: espero a que quede en reposo antes de reiniciar ROS"; ult_aviso=$espera
            fi
        else
            quieto=$(mas "$quieto" "$SONDEO_S")
        fi
        sleep "$SONDEO_S"
        espera=$(mas "$espera" "$SONDEO_S")
    done
}

activo_desde() {
    local ts
    ts=$(systemctl show agrobot-ros.service -p ActiveEnterTimestampMonotonic --value 2>/dev/null)
    [ -n "$ts" ] && [ "$ts" != "0" ] || { echo 0; return; }
    echo $(( ( $(cut -d. -f1 /proc/uptime) * 1000000 - ts ) / 1000000 ))
}

principal() {
    if sincronizado; then
        echo "[vigia-reloj] reloj ya sincronizado al arrancar — nada que hacer"
        exit 0
    fi
    echo "[vigia-reloj] reloj sin sincronizar ($(date)) — esperando a NTP"
    until sincronizado; do sleep 5; done
    echo "[vigia-reloj] NTP sincronizo ($(date))."
    if systemctl is-active --quiet agrobot-ros.service; then
        # NO reiniciar a media subida (2026-10-02, dos veces): con NTP a los pocos
        # segundos del arranque, el reinicio caía mientras los nodos se configuraban;
        # slam_toolbox y controller_server morían (-6/-11), el relanzado quedaba
        # colgado en "configurando" y Nav2 abortaba sin transform map. Se espera a
        # que ROS lleve ACTIVO_MIN_S activo y se hace parada completa + arranque.
        while [ "$(activo_desde)" -lt "$ACTIVO_MIN_S" ]; do sleep 5; done
        esperar_reposo
        echo "[vigia-reloj] reiniciando agrobot-ros para que TF arranque con el reloj correcto ($(date))."
        systemctl stop agrobot-ros.service
        # Que no quede ningún nodo del lanzamiento anterior (respawn incluido).
        for _ in $(seq 1 20); do
            pgrep -f "slam_toolbox|nav2_|sllidar_node|ekf_node|rf2o" > /dev/null || break
            sleep 1
        done
        systemctl start agrobot-ros.service
    fi
}

# VIGIA_LIB=1: solo define las funciones (pruebas)
[ "${VIGIA_LIB:-0}" = "1" ] || principal
