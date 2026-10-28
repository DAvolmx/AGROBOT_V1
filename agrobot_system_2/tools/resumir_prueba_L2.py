"""Resume la 2.ª prueba en el L (2026-10-28, giro de lazo cerrado de 12°): pasadas de la misión, especie, anticolisión, parte del mando manual, y el ángulo REAL por pulso de giro
(giroscopio y rumbo del EKF) para compararlo con la 1.ª prueba (mediana 1,5° / 2,4° con pulsos de 6°). Lee ~/.agrobot/datos/<sesión>; guarda todo en ~/.agrobot/pruebas/L2_20261028/.
Uso: python3 tools/resumir_prueba_L2.py <carpeta_de_la_sesion_de_datos>"""
import csv, json, sys
from collections import Counter
from pathlib import Path

import numpy as np

D = Path(sys.argv[1]); DEST = Path.home() / ".agrobot" / "pruebas" / "L2_20261028"; DEST.mkdir(parents=True, exist_ok=True)
R = [json.loads(l) for l in open(D / "eventos.jsonl") if l.startswith("{")]
corridas, cur = [], None
for r in R:
    if r["tipo"] != "mision_plantas":
        continue
    d, t = r["datos"], r["datos"].get("tipo")
    if t == "mision_inicio":
        cur = {"inicio_s": round(r["t"]), "plantas": [], "detecciones": 0, "fallidas": 0}; corridas.append(cur)
    elif cur is None:
        continue
    elif t == "planta_detectada": cur["detecciones"] += 1
    elif t == "inspeccion_fallida": cur["fallidas"] += 1
    elif t == "planta_inspeccionada": cur["plantas"].append({k: d.get(k) for k in ("n", "x", "y", "especie", "p_col")})
    elif t == "mision_fin": cur["fin_s"] = round(r["t"]); cur["motivo_fin"] = d.get("motivo")
est = Counter()
for r in R:
    if r["tipo"] == "exploracion":
        e = str(r["datos"].get("estado"))
        est["mando manual" if e.startswith("en pausa") else "nicho" if "nicho" in e else "callejón" if "callej" in e else "esquina" if e.startswith("esquina") else "guardián/sendero" if e.startswith(("guardián", "sendero")) else "girando" if e.startswith("girando") else "avanzando" if e.startswith("avanzando") else "detenido" if e.startswith("detenido") else "otro"] += 1
toasts = Counter(str(r["datos"].get("title")) for r in R if r["tipo"] == "toast")
plag = Counter(str(r["datos"].get("message")).split(" · ")[0] for r in R if r["tipo"] == "toast" and "Plaga" in str(r["datos"].get("title")))
# ángulos por pulso de giro
lineas = [l for l in open(D / "telemetria.csv", errors="replace") if "\x00" not in l]        # un corte de energía deja la cola del CSV llena de ceros (28-oct)
nul = sum("\x00" in l for l in open(D / "telemetria.csv", errors="replace"))
tel = [r for r in csv.DictReader(lineas) if r.get("t") not in (None, "")]
f = lambda k: np.array([float(r[k]) if r.get(k) not in ("", None) else np.nan for r in tel])
t, v, w, gz, rum = f("t"), f("v_aplicada"), f("w_aplicada"), f("giro_z"), f("rumbo_deg")
dt = np.gradient(t); pulsos = []; i = 0
while i < len(t):
    if abs(w[i]) > 0.1 and abs(v[i]) < 0.05:
        j = i
        while j + 1 < len(t) and abs(w[j + 1]) > 0.1 and t[j + 1] - t[j] < 0.6:
            j += 1
        k = min(len(t) - 1, j + 4); sg = np.sign(w[i])
        pulsos.append((t[j] - t[i] + 0.15, sg * float(np.nansum(gz[i:k + 1] * dt[i:k + 1])), sg * float((rum[k] - rum[max(i - 1, 0)] + 180) % 360 - 180))); i = j + 1
    else:
        i += 1
P = np.array(pulsos) if pulsos else np.zeros((0, 3))
res = lambda a: {"mediana_deg": round(float(np.median(a)), 1), "p25": round(float(np.percentile(a, 25)), 1), "p75": round(float(np.percentile(a, 75)), 1), "menos_de_2deg_pct": round(100 * float(np.mean(np.abs(a) < 2)))} if len(a) else None
out = {"sesion": D.name, "corridas": corridas, "estados_explorador": dict(est), "avisos_plaga": dict(plag), "anticolision": toasts.get("Anticolisión", 0), "paradas_emergencia": toasts.get("PARADA DE EMERGENCIA", 0),
       "giros": {"pulsos": int(len(P)), "duracion_mediana_s": round(float(np.median(P[:, 0])), 2) if len(P) else None, "giroscopio": res(P[:, 1]) if len(P) else None, "rumbo_ekf": res(P[:, 2]) if len(P) else None}}
(DEST / "resumen_L2.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
tot = ok = 0
for c in corridas:
    for p in c["plantas"]:
        tot += 1; ok += (p["especie"] == "col") == (p["p_col"] >= 0.5)
print("telemetría: filas válidas", len(tel), "| líneas con ceros descartadas", nul); print("sesión", D.name, "| pasadas", len(corridas), "| escaneos", tot, "| especie correcta", f"{ok}/{tot}")
for k, c in enumerate(corridas, 1):
    print(f"  pasada {k}: {c['inicio_s']}–{c.get('fin_s')} s ({c.get('motivo_fin')}) detecciones {c['detecciones']} fallidas {c['fallidas']} escaneos {[(p['especie'], p['p_col']) for p in c['plantas']]}")
print("estados:", dict(est)); print("avisos de plaga:", dict(plag), "| anticolisión:", out["anticolision"], "| emergencias:", out["paradas_emergencia"])
print("giros (pulsos de 12°):", out["giros"])
