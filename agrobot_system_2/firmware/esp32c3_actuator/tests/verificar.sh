#!/usr/bin/env bash
# Verificación completa del firmware SIN placa y SIN arduino-cli.
#
#   ./verificar.sh
#
# 1. Chequea la SINTAXIS del .ino contra los dos cores de ESP32 (2.x y 3.x),
#    que tienen APIs de LEDC distintas — el firmware soporta los dos y hay
#    que comprobar las dos ramas del #if, no solo la que use tu IDE.
# 2. Corre motor_logic_test.cpp: cinemática mecanum, zona muerta y arranque
#    por pulsos, contrastados contra el sketch que ya funciona en el robot.
#
# Los headers de stubs_arduino/ son cáscaras vacías: no simulan el hardware,
# solo permiten que g++ analice el sketch. Un OK acá NO garantiza que el
# robot se mueva; garantiza que compila y que la lógica da lo esperado.
set -e
cd "$(dirname "$0")"
for V in 2 3; do
  cp ../esp32c3_actuator.ino /tmp/_agrobot_sk.cpp
  g++ -fsyntax-only -std=gnu++17 -include stubs_arduino/Arduino.h \
      -DESP_ARDUINO_VERSION_MAJOR=$V -Istubs_arduino /tmp/_agrobot_sk.cpp
  echo "core ${V}.x -> compila"
done
g++ -std=c++17 -o /tmp/_agrobot_mlt motor_logic_test.cpp && /tmp/_agrobot_mlt
