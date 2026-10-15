# Entrega y prueba de banco

Directorio en Jetson: `/home/jetson/Desktop/AGROBOT/agrobot_system_2/firmware/esp32c3_pasivo_v4_codex_20261011`.

1. Ejecutar `python3 verificar_preparacion.py` para diagnóstico de lectura, sin enviar mandos.
2. Ejecutar `g++ -I. -std=c++11 tests/test_control.cpp -o /tmp/agrobot_v4_test && /tmp/agrobot_v4_test`.
3. Compilar: `~/.local/bin/arduino-cli compile --fqbn esp32:esp32:esp32c3 --output-dir build .`.
4. El .ino es autónomo. `ControlPasivo.h` reproduce el núcleo para pruebas; el generador local lo inserta dentro del .ino. Si se modifican parámetros en uno, sincronizar ambos antes de repetir las pruebas.
5. Cargar sólo después de identificar placa/puerto y tener las ruedas levantadas. Verificar que reinicio, neutro, timeout y desconexión no activan motores. El perfil de esta entrega permanece bloqueado.
6. Confirmar mapa real de los cuatro canales y modelo de puente H. Ajustar vía entre centros y calibración PWM bajo carga. Sólo entonces habilitar PROFILE_VERIFIED y probar avance corto, curva suave, neutro y pérdida de paquetes, con parada accesible. Nunca empezar con giro sobre el sitio.

No se ha cambiado el servicio activo de Claude ni flasheado la C3. El backend ya existente envía neutro y necesita recibir salida de failsafe antes de permitir movimiento. Su reinicio de secuencia requiere manejo explícito si la C3 no se reinicia: contador modular no implica una sesión nueva. El firmware no tiene autenticación criptográfica, encoders ni sensor del pivote.

Cuellos de botella: flash de partición estándar cerca del 95% por BLE+WiFi, margen reducido para nuevas bibliotecas; procesamiento abierto sin medida de velocidad; tabla PWM-driver pendiente; inferencia Jetson y firmware no deben confundirse. No se desactivaron comprobaciones para ganar velocidad.

La batería C++ verifica el mezclador; no certifica transporte ni respuesta física. El AP y proceso del backend activos no prueban que la placa esté conectada por BLE/WiFi. No usar el estado de un firmware anterior como evidencia de instalación de esta versión.
