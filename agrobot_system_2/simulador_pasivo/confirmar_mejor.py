"""Confirmación SIN SESGO del mejor candidato: otro juego de semillas que no se vio ni al entrenar ni al elegirlo (el elegido salió de varios candidatos
validados, así que su validación es algo optimista). Compara mejor vs HOY vs AYER, con y sin hardware flojo. Salida: confirmacion.json junto al informe."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
import entrenar_intensivo as T

inf = Path(sys.argv[1])
d = json.loads(inf.read_text())
cands = {"mejor": d["mejor"]["params"], "HOY": T.HOY, "AYER": T.AYER}
with Pool(6) as pool:
    normal = T.evaluar(pool, list(cands.values()), list(range(9000, 9024)), list(range(9500, 9512)))
    flojo = T.evaluar(pool, list(cands.values()), list(range(9100, 9112)), list(range(9600, 9606)), extremo=True)
out = {"normal": {n: T.resumen(r) for n, r in zip(cands, normal)}, "hardware_flojo": {n: T.resumen(r) for n, r in zip(cands, flojo)}}
(inf.parent / "confirmacion.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
for k, v in out.items():
    for n, r in v.items():
        print(k, n, r["todo"], flush=True)
