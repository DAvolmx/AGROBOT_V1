"""Mide si el ancla de rumbo por paredes de `mapa_scan.MapaScan` (manhattan) reduce la deriva del mapa dibujado.

Mismo banco que validar_digitalizacion.py (10 mapas de verdad conocida, pose con deriva) pero el mapa sale de MapaScan (el del dashboard):
con el ancla apagada y encendida, se digitaliza y se compara con los muros reales. Uso: python3 validar_mapa_scan.py [--semillas 3]"""
import argparse, base64, json, math, sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent / "jetson" / "backend"))
import digitalizar_mapa as DM          # noqa: E402
import mapa_scan as MS                 # noqa: E402
import mapas_interiores as MI          # noqa: E402
import sim_logica_actual as S          # noqa: E402
import validar_digitalizacion as VD    # noqa: E402


def mapa_con_scan(e, sp, sh, rng, manhattan):
    m = MS.MapaScan(manhattan=manhattan)
    tray = VD.recorrido(e)
    tray = tray + tray[::-1][1:]
    ex = ey = eh = 0.0
    prev = tray[0]
    t = 0.0
    for (x, y, th) in tray:
        paso = math.hypot(x - prev[0], y - prev[1])
        ex += rng.normal(0, sp * math.sqrt(paso)); ey += rng.normal(0, sp * math.sqrt(paso))
        eh += rng.normal(0, math.radians(sh) * math.sqrt(paso))
        prev = (x, y, th)
        t += 0.1
        scan = S.lidar_scan(e, x, y, th, rng)
        if th != tray[0][2] and False:
            pass
        m.agregar(t, {"x": x + ex, "y": y + ey, "theta_deg": math.degrees(th + eh)},
                  [r["angle_deg"] for r in scan], [r["distance_m"] for r in scan])
    mp = m.como_mapa()
    if mp is None:
        return None, m
    g = np.frombuffer(base64.b64decode(mp["data_b64"]), np.int8).reshape(mp["height"], mp["width"])
    return (np.flipud(g), mp["resolution"], (mp["origin_x"], mp["origin_y"])), m


def evaluar(nombre, e, sp, sh, semilla, manhattan, tol=0.10):
    rng = np.random.default_rng(semilla)
    mapa, m = mapa_con_scan(e, sp, sh, rng, manhattan)
    if mapa is None:
        return None
    r = DM.digitalizar(mapa)
    ext, real = r["muros"], [list(s) for s in e.paredes]
    Pr, Pe = VD.muestrear(real), VD.muestrear(ext)
    tray = np.array([(x, y) for x, y, _ in VD.recorrido(e)])
    vis = np.array([np.min(np.hypot(tray[:, 0] - p[0], tray[:, 1] - p[1])) <= 1.5 for p in Pr])
    dr = VD.dist_a_segmentos(Pr[vis], ext)
    de = VD.dist_a_segmentos(Pe, real)
    return {"mapa": nombre, "manhattan": manhattan, "recall": 100 * float((dr <= tol).mean()), "precision": 100 * float((de <= tol).mean()) if len(de) else 0.0,
            "error_cm": 100 * float(np.mean(np.minimum(de, 0.5))) if len(de) else 50.0, "anclajes": m.anclajes}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semillas", type=int, default=3)
    a = ap.parse_args()
    out = []
    for nivel, sp, sh in (("deriva_1pct", 0.012, 0.5), ("deriva_3pct", 0.03, 1.5)):
        for man in (False, True):
            fs = [f for n, e in MI.mapas().items() for s in range(a.semillas) if (f := evaluar(n, e, sp, sh, s, man)) is not None]
            for f in fs:
                f["nivel"] = nivel
            out += fs
            print(f"{nivel:12s} ancla={'sí' if man else 'no':2s}  recall {np.mean([f['recall'] for f in fs]):5.1f}  precisión {np.mean([f['precision'] for f in fs]):5.1f}  "
                  f"error {np.mean([f['error_cm'] for f in fs]):4.1f} cm  anclajes/mapa {np.mean([f['anclajes'] for f in fs]):.0f}", flush=True)
    Path(AQUI.parent / "docs" / "digitalizacion" / "mapa_scan_ancla.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
