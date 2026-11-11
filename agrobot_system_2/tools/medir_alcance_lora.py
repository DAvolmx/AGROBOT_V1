#!/usr/bin/env python3
"""Mide si los uplinks LoRaWAN del nodo del robot llegan al gateway HT-M02 desde lejos.

Se corre en la Jetson durante la prueba (p. ej. huerto de arándano a 2-3 km del gateway):

    python3 tools/medir_alcance_lora.py --punto "huerto 2.5 km" --minutos 10

Cada 5 s:
  - lee el histórico de la Pi (`/api/history`, puerto 8092) por Tailscale, o por LAN si está en casa,
    y anota cada uplink NUEVO del nodo real (DevEUI f59d08345985b27d; ignora los simulados 51a7…)
    con su RSSI, SNR y SF;
  - mira en el backend local si el nodo sigue vivo (salud `lora_v3`, por UDP/Wi-Fi del robot). Así
    "no llegó nada" se distingue de "el nodo estaba apagado".

Al final imprime recibidos/esperados (el nodo manda cada 30 s), y RSSI/SNR mínimo, mediana y máximo.
Todo queda en ~/.agrobot/pruebas/alcance_lora_<fecha>/<punto>.csv.

Sin internet en el huerto no hay Tailscale: el script sigue registrando si el nodo estuvo vivo y,
al volver, `--recuperar` baja de la Pi los uplinks del intervalo (la Pi los guarda en SQLite).
Solo usa la biblioteca estándar.
"""
import argparse
import csv
import json
import statistics
import time
import urllib.request
from datetime import datetime
from pathlib import Path

DEV_EUI = "f59d08345985b27d"
PERIODO_NODO_S = 30
PIS = ["http://100.121.219.100:8092", "http://192.168.0.10:8092", "http://192.168.0.20:8092"]
BACKEND = "http://127.0.0.1:8080"


def get_json(url, timeout=4):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def historial_pi(limit=60):
    for base in PIS:
        try:
            return base, get_json(f"{base}/api/history?limit={limit}", timeout=2)
        except Exception:
            continue
    return None, None


def nodo_vivo():
    try:
        for x in get_json(f"{BACKEND}/api/health", timeout=2):
            if x.get("key") == "lora_v3":
                return x.get("status") == "ok", x.get("age_s")
    except Exception:
        pass
    return None, None


def resumen(filas, t0, t1, punto):
    up = [f for f in filas if f["tipo"] == "uplink"]
    esperados = max(1, int((t1 - t0) // PERIODO_NODO_S))
    vivos = [f for f in filas if f["tipo"] == "muestra" and f["nodo_vivo"] == "si"]
    muestras = [f for f in filas if f["tipo"] == "muestra"]
    sin_pi = [f for f in muestras if not f["pi"]]
    print(f"\n=== {punto}: {len(up)} uplinks recibidos de ~{esperados} esperados "
          f"({100 * len(up) / esperados:.0f} %) en {(t1 - t0) / 60:.1f} min ===")
    print(f"nodo vivo en {len(vivos)}/{len(muestras)} muestras; Pi inalcanzable en {len(sin_pi)}/{len(muestras)}")
    for k in ("rssi", "snr"):
        v = [float(f[k]) for f in up if f[k] not in ("", None)]
        if v:
            print(f"{k.upper():4s} min {min(v):.1f}  mediana {statistics.median(v):.1f}  max {max(v):.1f}")
    sfs = sorted({f["sf"] for f in up if f["sf"]})
    if sfs:
        print("SF usados:", ", ".join(map(str, sfs)))
    if not up and sin_pi and len(sin_pi) == len(muestras):
        print("La Pi no respondió nunca: sin VPN no se sabe aún. Al volver: --recuperar con este CSV.")


def medir(a):
    carpeta = Path.home() / ".agrobot/pruebas" / f"alcance_lora_{datetime.now():%Y%m%d}"
    carpeta.mkdir(parents=True, exist_ok=True)
    nombre = "".join(c if c.isalnum() or c in "-_." else "_" for c in a.punto)
    ruta = carpeta / f"{nombre}_{datetime.now():%H%M%S}.csv"
    campos = ["t", "hora", "punto", "tipo", "pi", "nodo_vivo", "edad_nodo_s", "rssi", "snr", "sf", "id_pi"]
    vistos, filas = set(), []
    base, h = historial_pi()
    if h:  # lo que ya estaba antes de empezar no cuenta
        vistos = {p["id"] for p in h}
    t0 = time.time()
    fin = t0 + a.minutos * 60
    print(f"Midiendo '{a.punto}' {a.minutos} min -> {ruta}")
    with open(ruta, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=campos)
        w.writeheader()
        while time.time() < fin:
            now = time.time()
            vivo, edad = nodo_vivo()
            base, h = historial_pi()
            fila = {"t": f"{now:.1f}", "hora": datetime.now().strftime("%H:%M:%S"), "punto": a.punto,
                    "tipo": "muestra", "pi": base or "", "nodo_vivo": {True: "si", False: "no"}.get(vivo, "?"),
                    "edad_nodo_s": "" if edad is None else f"{edad:.1f}", "rssi": "", "snr": "", "sf": "", "id_pi": ""}
            w.writerow(fila); filas.append(fila)
            for p in sorted(h or [], key=lambda p: p["id"]):
                if p["id"] in vistos or p.get("dev_eui") != DEV_EUI:
                    continue
                vistos.add(p["id"])
                u = dict(fila, tipo="uplink", t=f"{p['received_at']:.1f}",
                         hora=datetime.fromtimestamp(p["received_at"]).strftime("%H:%M:%S"),
                         rssi=p.get("rssi"), snr=p.get("snr"), sf=p.get("sf"), id_pi=p["id"])
                w.writerow(u); filas.append(u)
                print(f"{u['hora']}  UPLINK  RSSI {u['rssi']} dBm  SNR {u['snr']} dB  SF{u['sf']}")
            fh.flush()
            if not (h is not None):
                print(f"{fila['hora']}  Pi inalcanzable (¿VPN?)  nodo vivo: {fila['nodo_vivo']}")
            time.sleep(5)
    resumen(filas, t0, time.time(), a.punto)


def recuperar(ruta_csv):
    """Con la Pi ya alcanzable, completa un CSV medido sin VPN con los uplinks de su intervalo."""
    filas = list(csv.DictReader(open(ruta_csv)))
    t0, t1 = float(filas[0]["t"]), float(filas[-1]["t"]) + 5
    base, h = historial_pi(limit=2000)
    if not h:
        raise SystemExit("La Pi sigue sin responder.")
    ya = {f["id_pi"] for f in filas if f["tipo"] == "uplink"}
    nuevos = [p for p in h if p.get("dev_eui") == DEV_EUI and t0 <= p["received_at"] <= t1 and str(p["id"]) not in ya]
    for p in nuevos:
        filas.append({**{k: "" for k in filas[0]}, "t": f"{p['received_at']:.1f}", "punto": filas[0]["punto"],
                      "hora": datetime.fromtimestamp(p["received_at"]).strftime("%H:%M:%S"), "tipo": "uplink",
                      "pi": base, "rssi": p.get("rssi"), "snr": p.get("snr"), "sf": p.get("sf"), "id_pi": p["id"]})
    filas.sort(key=lambda f: float(f["t"]))
    with open(ruta_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(filas[0].keys()))
        w.writeheader(); w.writerows(filas)
    print(f"{len(nuevos)} uplinks recuperados de {base}")
    resumen(filas, t0, t1, filas[0]["punto"])


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--punto", default="sin_nombre", help="lugar y distancia al gateway, p. ej. 'huerto 2.5 km'")
    ap.add_argument("--minutos", type=float, default=10)
    ap.add_argument("--recuperar", metavar="CSV", help="completar un CSV medido sin VPN")
    a = ap.parse_args()
    recuperar(a.recuperar) if a.recuperar else medir(a)
