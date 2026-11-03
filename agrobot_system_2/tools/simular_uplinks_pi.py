"""Uplinks LoRaWAN SIMULADOS hacia el panel de la Raspberry Pi, para demostraciones (el SCD40 del nodo está quemado y el nodo no transmite).

Publica en el MQTT de ChirpStack de la Pi (application/<app>/device/<devEui>/event/up) el mismo JSON que publica ChirpStack, con el payload de
17 bytes del firmware v3 (decoder.py). Así recorren el camino real: gateway_server.on_mqtt_message → SQLite → WebSocket → dashboard.
Usa un DevEUI PROPIO (por defecto 51a7000000000001) para que nunca se confundan con datos reales; se borran con:
    sqlite3 <base de la Pi> "DELETE FROM packets WHERE dev_eui='51a7000000000001'"
Plan AU915 (canales 0–7: 915,2–916,6 MHz, banda de uso libre 915–928 MHz según la nota EQA.45 del PNF de ARCOTEL).
Valores dentro de rango (no disparan alarmas de «fuera de rango»): T 21–27 °C, HR 55–80 %, CO2 420–750 ppm, IMU en reposo.
Uso: python3 tools/simular_uplinks_pi.py [--host 100.121.219.100] [--cada 20] [--horas 6] [--rafaga 40]
"""
import argparse, base64, json, math, random, struct, time

import paho.mqtt.client as mqtt

APP = "aa4c5278-0d39-4833-be36-286895ffa4c4"        # CHIRPSTACK_APP_ID de rpi/server/gateway_server.py


def payload_v3(t, rng):
    """17 bytes big-endian: accel×100 (3·int16), gyro×10 (3·int16), co2 uint16, tempSCD×100 int16, humedad uint8."""
    fase = 2 * math.pi * (t % 86400) / 86400
    temp = 24.0 + 2.5 * math.sin(fase - 1.2) + rng.gauss(0, 0.15)
    hum = 68 - 9 * math.sin(fase - 1.2) + rng.gauss(0, 0.8)
    co2 = 560 + 120 * math.cos(fase) + rng.gauss(0, 12)
    acc = [rng.gauss(0, 0.02), rng.gauss(0, 0.02), 1.0 + rng.gauss(0, 0.02)]
    gyr = [rng.gauss(0, 0.4) for _ in range(3)]
    return struct.pack(">hhhhhhHhB", *[int(round(a * 100)) for a in acc], *[int(round(g * 10)) for g in gyr],
                       int(round(co2)), int(round(temp * 100)), int(round(min(max(hum, 0), 100)))), (temp, hum, co2)


def evento(dev, raw, f_cnt, rng):
    return {"deduplicationId": f"sim-{f_cnt}", "time": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "deviceInfo": {"applicationId": APP, "deviceName": "nodo-SIMULADO", "devEui": dev, "tags": {"simulado": "si"}},
            "fCnt": f_cnt, "fPort": 2, "data": base64.b64encode(raw).decode(),
            "rxInfo": [{"gatewayId": "065f31fffe49b5a6", "rssi": int(round(rng.gauss(-68, 4))), "snr": round(rng.gauss(10, 1.5), 1)}],
            "txInfo": {"frequency": 915200000 + 200000 * (f_cnt % 8), "modulation": {"lora": {"bandwidth": 125000, "spreadingFactor": 7, "codeRate": "CR_4_5"}}}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.10")
    ap.add_argument("--dev", default="51a7000000000001")
    ap.add_argument("--cada", type=float, default=20.0, help="segundos entre paquetes (el panel marca obsoleto a los 90 s)")
    ap.add_argument("--horas", type=float, default=6.0)
    ap.add_argument("--rafaga", type=int, default=40, help="paquetes iniciales cada 1,5 s para que las gráficas tengan curva")
    a = ap.parse_args()
    rng = random.Random(7)
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="agrobot-simulador-uplinks")
    c.connect(a.host, 1883, 30); c.loop_start()
    topic = f"application/{APP}/device/{a.dev}/event/up"
    fin, n, t_sim = time.time() + a.horas * 3600, 0, time.time() - a.rafaga * 600   # la ráfaga recorre ~las últimas horas del ciclo diario
    while time.time() < fin:
        n += 1
        t_val = t_sim + n * 600 if n <= a.rafaga else time.time()
        raw, (T, H, C) = payload_v3(t_val, rng)
        c.publish(topic, json.dumps(evento(a.dev, raw, n, rng)), qos=1).wait_for_publish(10)
        print(time.strftime("%H:%M:%S"), f"#{n} T={T:.1f} HR={H:.0f} CO2={C:.0f}", flush=True)
        time.sleep(1.5 if n <= a.rafaga else a.cada)
    c.loop_stop(); c.disconnect()


if __name__ == "__main__":
    main()
