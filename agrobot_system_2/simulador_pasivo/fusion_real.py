#!/usr/bin/env python3
"""
Fusión entrenamiento SIMULACIÓN + REAL.
=======================================

El entrenamiento (`entrenar.py`) aprende SOLO del piloto ideal del simulador
(`dataset.csv`, orígenes `ideal*`). Pero el robot real graba cada sesión en
`~/.agrobot/datos/<fecha>/telemetria.csv` (ver `grabador.py`), con lo que
percibía y la acción que se tomó. Esos recorridos reales son demostraciones
valiosas que hoy NO entran al entrenamiento.

Este conversor cierra esa brecha: lee las `telemetria.csv` reales, las traduce
a las MISMAS 14 columnas del dataset (`politica.ENTRADAS` + accion/escenario/
origen) y las deja listas para sumarlas. Así la política se entrena
CORRELACIONANDO lo aprendido en simulación con lo que de verdad funcionó (o lo
que el operador corrigió) en el campo.

Qué puede y qué no (honesto):
  · La telemetría real trae los 4 sectores del LIDAR, v y omega → 10 de las 14
    features salen directas. Las 4 de SENDERO no se graban en la telemetría, así
    que entran como "no visible" (sendero_visible=0), igual que muchos tramos de
    simulación sin sendero. Es una limitación, no un error: el aporte real es
    sobre todo navegación por LIDAR y las correcciones del operador.
  · Se etiqueta el ORIGEN: `real_manual` (el operador conducía: demostración de
    oro) y `real_auto` (condujo la autonomía y no chocó). Así `entrenar.py`
    puede pesarlos distinto.
  · Se DESCARTA lo que no enseña: filas quietas, con failsafe del C3, o con
    acción no estándar.

Uso:
    python3 simulador/fusion_real.py                 # DRY-RUN: analiza y reporta
    python3 simulador/fusion_real.py --escribir      # añade al dataset.csv real
    python3 simulador/fusion_real.py --datos DIR --salida out.csv
"""
import argparse
import csv
import math
import sys
from collections import Counter
from pathlib import Path

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))

import politica as pol   # noqa: E402

RAIZ_DATOS = Path.home() / ".agrobot" / "datos"
DATASET = Path.home() / ".agrobot" / "aprendizaje" / "dataset.csv"
COLUMNAS = list(pol.ENTRADAS) + ["accion", "escenario", "origen"]

ACCIONES = set(pol.ACCIONES)              # adelante, izquierda, derecha, atras
# Un tramo real muy corto o quieto no enseña a conducir.
V_MIN_MOV = 0.02


def _f(v):
    """Celda de CSV -> float o None."""
    if v is None or v == "":
        return None
    try:
        x = float(v)
        return None if math.isnan(x) else x
    except ValueError:
        return None


def fila_a_sensores(fila: dict) -> dict:
    """telemetria.csv -> el dict `sensores` que consume politica.caracteristicas.

    La telemetría ya trae las distancias del LIDAR en METROS por sector (lo que
    devuelve `LidarReactive.distancias`). Las de sendero no se graban: van None,
    y `caracteristicas` las trata como "no visible"."""
    v = _f(fila.get("v_c3"))
    if v is None:
        v = _f(fila.get("v_aplicada"))
    w = _f(fila.get("w_c3"))
    if w is None:
        w = _f(fila.get("w_aplicada"))
    return {
        "lidar_adelante_m": _f(fila.get("lidar_adelante")),
        "lidar_atras_m": _f(fila.get("lidar_atras")),
        "lidar_izquierda_m": _f(fila.get("lidar_izquierda")),
        "lidar_derecha_m": _f(fila.get("lidar_derecha")),
        "sendero_lateral_m": None, "sendero_rumbo_deg": None,
        "sendero_ancho_m": None, "sendero_confianza": None,
        "v_actual": v, "omega_actual": w,
    }


def origen_de(fila: dict) -> str:
    modo = (fila.get("modo_manejo") or "").lower()
    # Si conducía la persona (manual/preciso/interior/exterior sin autonomía) es
    # demostración del operador. El grabador no marca "auto" explícito en modo,
    # así que se usa la columna de acción del pulso + el modo como pista.
    if modo in ("manual", "preciso", "interior", "exterior"):
        return "real_manual"
    return "real_auto"


def convertir_sesion(csv_path: Path):
    """Lee una telemetria.csv -> (filas_dataset, descartes)."""
    filas, descartes = [], Counter()
    try:
        with csv_path.open(newline="") as f:
            for fila in csv.DictReader(f):
                accion = (fila.get("pulso_accion") or "").strip()
                if accion not in ACCIONES:
                    descartes["sin_accion"] += 1
                    continue
                if str(fila.get("c3_failsafe")).lower() in ("true", "1"):
                    descartes["failsafe"] += 1
                    continue
                sensores = fila_a_sensores(fila)
                v = abs(sensores["v_actual"] or 0.0)
                w = abs(sensores["omega_actual"] or 0.0)
                if v < V_MIN_MOV and w < V_MIN_MOV:
                    descartes["quieto"] += 1
                    continue
                if all(sensores[k] is None for k in
                       ("lidar_adelante_m", "lidar_izquierda_m", "lidar_derecha_m")):
                    descartes["sin_lidar"] += 1
                    continue
                feats = pol.caracteristicas(sensores)
                filas.append(feats + [accion, csv_path.parent.name, origen_de(fila)])
    except OSError as e:
        descartes["error_lectura"] += 1
        print(f"  ! {csv_path}: {e}")
    return filas, descartes


def main():
    p = argparse.ArgumentParser(description="Fusiona telemetría real al dataset de entrenamiento")
    p.add_argument("--datos", default=str(RAIZ_DATOS), help="carpeta con <fecha>/telemetria.csv")
    p.add_argument("--salida", default=str(DATASET), help="dataset destino (se AÑADE)")
    p.add_argument("--escribir", action="store_true",
                   help="añade de verdad al dataset (sin esto: solo analiza)")
    a = p.parse_args()

    raiz = Path(a.datos)
    sesiones = sorted(raiz.glob("*/telemetria.csv"))
    if not sesiones:
        print(f"No hay telemetria.csv en {raiz}")
        return

    todas, descartes = [], Counter()
    por_sesion = []
    for csv_path in sesiones:
        filas, d = convertir_sesion(csv_path)
        todas.extend(filas)
        descartes.update(d)
        if filas:
            por_sesion.append((csv_path.parent.name, len(filas)))

    origenes = Counter(r[-1] for r in todas)
    acciones = Counter(r[-3] for r in todas)
    print(f"Sesiones con datos útiles: {len(por_sesion)}/{len(sesiones)}")
    print(f"Filas de demostración real: {len(todas)}")
    print(f"  por origen : {dict(origenes)}")
    print(f"  por acción : {dict(acciones)}")
    print(f"  descartadas: {dict(descartes)}")

    # Correlación con el dataset de simulación: distribución de acciones.
    salida = Path(a.salida)
    if salida.exists():
        sim = Counter()
        with salida.open(newline="") as f:
            for fila in csv.DictReader(f):
                sim[fila["accion"]] += 1
        tot_sim = sum(sim.values()) or 1
        tot_real = sum(acciones.values()) or 1
        print("\nCorrelación acción (sim % vs real %):")
        for acc in pol.ACCIONES:
            print(f"  {acc:10s} sim {100*sim.get(acc,0)/tot_sim:5.1f}%   "
                  f"real {100*acciones.get(acc,0)/tot_real:5.1f}%")

    if not a.escribir:
        print("\n(DRY-RUN) No se escribió nada. Añadí --escribir para sumar al dataset.")
        return

    nuevo = not salida.exists()
    with salida.open("a", newline="") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(COLUMNAS)
        w.writerows(todas)
    print(f"\n✅ Añadidas {len(todas)} filas reales a {salida}")
    print("   Para que entren al entrenar: incluí 'real_manual'/'real_auto' en "
          "ORIGENES de entrenar.py (o entrená con --todos-los-datos).")


if __name__ == "__main__":
    main()
