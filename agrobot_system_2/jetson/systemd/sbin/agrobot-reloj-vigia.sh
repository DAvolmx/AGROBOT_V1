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
sleep 3
systemctl try-restart agrobot-ros.service
