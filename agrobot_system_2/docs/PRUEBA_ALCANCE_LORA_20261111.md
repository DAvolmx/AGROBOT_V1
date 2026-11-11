# Prueba de alcance LoRa: huerto de arándano (2-3 km del gateway)

**Pregunta:** ¿llegan al gateway HT-M02 (casa, 192.168.0.11) los uplinks LoRaWAN del nodo Heltec V3 que va en el robot?

## Cómo viaja cada dato
- **Nodo Heltec V3 (en el robot):** manda un uplink LoRaWAN cada 30 s (AU915, OTAA, ADR) con DevEUI `f59d08345985b27d`.
- **En casa:** el HT-M02 reenvía a ChirpStack en la Pi. La Pi guarda RSSI/SNR/SF de cada paquete y los muestra en `:8092`.
- **Para verlo desde el huerto:** por Tailscale.
  - Pi: `http://100.121.219.100:8092`
  - Robot: `http://100.82.220.28:8080`

## Antes de salir (en casa)
1. **La Pi `lora-server-pi` debe estar encendida y en red** (verificado el 2026-11-11 a las 14:45: en casa llegan 4/4 uplinks, RSSI −41 dBm, SNR 13,8 dB, SF7).
2. Abrir `http://100.121.219.100:8092` y confirmar que llegan uplinks con el robot en casa (RSSI de referencia).
3. Tailscale abierto en el celular o la laptop desde donde se va a mirar.

## En el huerto
1. **Internet para la Jetson:** conectar su Wi-Fi interno (`wlP1p1s0`) al hotspot del celular. El AP del robot va por la antena USB, así que no hay conflicto de canal. Sin esto no hay VPN, pero la medición igual se registra en la Pi.
2. Encender el robot **en el huerto**. El nodo hace OTAA de nuevo y arranca sin el SF7 que ADR le dejó en casa. Si no se une (join), tampoco va a haber uplinks: eso ya es un resultado.
3. Medir 10 min por punto, sin mover el robot:
   ```
   python3 tools/medir_alcance_lora.py --punto "huerto 2.5 km" --minutos 10
   ```
   Imprime cada uplink recibido y, al final, recibidos/esperados, RSSI/SNR y SF.
   Anotar el punto en el celular (GPS) para saber la distancia real al gateway.
4. Si no hubo VPN, al volver a casa: `python3 tools/medir_alcance_lora.py --recuperar ~/.agrobot/pruebas/alcance_lora_<fecha>/<punto>.csv`.

## Cómo leerlo
- **≥ 80 % recibidos:** enlace utilizable.
- **Algunos recibidos con SNR cerca de −15 a −20 dB:** límite del alcance.
- **0 recibidos con "nodo vivo = sí":** no hay enlace a esa distancia.
  - Probar de nuevo levantando el robot o el gateway.
  - La antena del HT-M02 adentro de la casa es lo que más limita.
- El nodo usa ADR. Si ya estaba unido en casa con SF7, ADR tarda ~45 min sin respuestas en bajar el DR: por eso conviene encenderlo allá.
