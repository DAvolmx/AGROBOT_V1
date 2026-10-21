#!/usr/bin/env python3
"""Mide la latencia de ida y vuelta del cable USB (CDC) Jetson <-> C3 con las tramas reales del protocolo (2026-10-21).

Manda órdenes (v=0, omega=0: el robot NO se mueve) a 20 Hz con número de secuencia creciente; la C3 contesta al instante con un
estado cuyo `seq_echo` es la secuencia que acaba de aplicar. RTT = instante del eco - instante del envío de esa secuencia.
  python3 tools/medir_latencia_c3.py [--segundos 20] [--hz 20] [--puerto /dev/serial/by-id/usb-Espressif_*]
NO usar con el backend conectado al mismo puerto (se pelean).
"""
import argparse
import glob
import struct
import sys
import time

import serial

CMD_MAGIC, STATUS_MAGIC = 0xA5, 0x5A
FMT = "<BbbBBHB"
ST_FAILSAFE, ST_USB_OK = 0x04, 0x80
FLAG_LED = 0x01            # el foco va invertido en el firmware: FLAG_LED puesto = foco APAGADO (ver uart_link.pack)


def crc8(b):
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--segundos", type=float, default=20)
    ap.add_argument("--hz", type=float, default=20)
    ap.add_argument("--puerto", default=None)
    a = ap.parse_args()
    puerto = a.puerto or (glob.glob("/dev/serial/by-id/usb-Espressif*") or ["/dev/ttyACM0"])[0]
    s = serial.Serial(puerto, 115200, timeout=0, dsrdtr=False, rtscts=False)
    s.dtr = False
    s.rts = False
    print("puerto", puerto, flush=True)
    time.sleep(1.5)
    s.reset_input_buffer()
    enviados, rtt, buf = {}, [], bytearray()
    flags_vistos, tramas = set(), 0
    seq, t0, prox = 100, time.perf_counter(), time.perf_counter()
    while time.perf_counter() - t0 < a.segundos + 0.5:
        ahora = time.perf_counter()
        if ahora - t0 < a.segundos and ahora >= prox:
            seq = (seq + 1) & 0xFFFF
            cuerpo = struct.pack(FMT[:-1], CMD_MAGIC, 0, 0, 98, FLAG_LED, seq)
            enviados[seq] = ahora
            s.write(cuerpo + bytes([crc8(cuerpo)]))
            prox += 1.0 / a.hz
        d = s.read(256)
        if d:
            buf += d
            t_rx = time.perf_counter()
            while len(buf) >= 8:
                i = buf.find(bytes([STATUS_MAGIC]))
                if i < 0:
                    buf.clear()
                    break
                if i > 0:
                    del buf[:i]
                    continue
                if len(buf) < 8:
                    break
                tr = bytes(buf[:8])
                if crc8(tr[:7]) != tr[7]:
                    del buf[0]
                    continue
                del buf[:8]
                _, _v, _w, _srv, fl, eco, _c = struct.unpack(FMT, tr)
                tramas += 1
                flags_vistos.add(fl)
                if eco in enviados and eco not in [e for e, _ in rtt]:
                    rtt.append((eco, (t_rx - enviados[eco]) * 1000.0))
        else:
            time.sleep(0.0005)
    s.close()
    n_env = len(enviados)
    if not rtt:
        print(f"SIN RESPUESTA: {n_env} órdenes enviadas, 0 ecos. ¿Firmware sin canal USB, o el backend usa el puerto?")
        sys.exit(1)
    v = sorted(x for _, x in rtt)
    pct = lambda p: v[min(len(v) - 1, int(p / 100 * len(v)))]
    print(f"órdenes enviadas {n_env} · ecos con su secuencia {len(rtt)} ({100 * len(rtt) / n_env:.1f} %) · estados totales {tramas}")
    print(f"RTT ida y vuelta [ms]: min {v[0]:.2f} · media {sum(v) / len(v):.2f} · p50 {pct(50):.2f} · p95 {pct(95):.2f} · p99 {pct(99):.2f} · máx {v[-1]:.2f}")
    print("flags de estado vistos:", sorted(hex(f) for f in flags_vistos), "(0x4 = failsafe, 0x80 = USB activo)")


if __name__ == "__main__":
    main()
