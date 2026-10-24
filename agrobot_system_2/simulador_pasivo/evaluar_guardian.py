"""Evalúa el guardián del sendero en el simulador calibrado: BASE (explorador de hoy) vs GUARDIÁN solo con la ley vs GUARDIÁN con memoria
entrenada (correlación LIDAR+IMU+cámara+error). 1) entrena la memoria en semillas 100+ explorando; 2) evalúa con semillas nuevas, cada episodio
con una copia congelada de la memoria. Salida: ../docs/guardian/resultados.json. Uso: PYTHONPATH=~/.agrobot/numba_pkg python3 evaluar_guardian.py"""
import copy, json, sys, time
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson" / "backend"))
import entorno_calibrado as EC
import guardian_sendero as GS

SAL = Path(__file__).resolve().parents[1] / "docs" / "guardian"


def _ep(a):
    ent, esc, modo, seed, g = a
    try:
        extra = {} if modo == "base" else {"guardian": True, "guardian_obj": g, "guardian_usar_memoria": modo == "memoria"}
        r = EC.correr(ent, esc, EC.config_base(ent, **extra), seed)
        out = {k: r[k] for k in ("exito", "contactos", "progreso", "rmse_cm", "pico_cm", "correcciones", "t_s")}
        if "guardian" in r:
            out["guardian"] = r["guardian"]
        return dict(out, ent=ent, esc=esc, modo=modo, seed=seed)
    except Exception as e:                                   # noqa: BLE001
        return {"ent": ent, "esc": esc, "modo": modo, "seed": seed, "error": repr(e), "exito": False, "contactos": 9, "progreso": 0.0,
                "rmse_cm": 99.0, "pico_cm": 99.0, "correcciones": 0, "t_s": 0}


def main():
    t0 = time.time()
    SAL.mkdir(parents=True, exist_ok=True)
    esc = [(e, n) for e in ("interior", "exterior") for n in EC.escenarios(e)]
    # 1) entrenamiento de la memoria (un proceso: la memoria se comparte entre episodios)
    mem = GS.MemoriaCorrecciones(SAL / "memoria_entrenada.json", max_casos=4000)
    g = GS.GuardianSendero(mem, log_dir=SAL / "registro", explorar=0.2, usar_memoria=True, min_casos=4)
    for seed in range(100, 104):
        for e, n in esc:
            r = _ep((e, n, "memoria", seed, g))
        print(f"entrenamiento semilla {seed}: casos {len(mem.casos)} · {time.time()-t0:.0f}s", flush=True)
    mem.guardar()
    g.explorar = 0.0
    # 2) evaluación con semillas nuevas
    trabajos = [(e, n, m, s, copy.deepcopy(g) if m != "base" else None) for e, n in esc for s in range(3) for m in ("base", "ley", "memoria")]
    for t in trabajos:
        if t[4] is not None:
            t[4].log_dir = None
            t[4]._log = None
    with Pool(2) as p:
        res = p.map(_ep, trabajos, chunksize=1)
    (SAL / "resultados.json").write_text(json.dumps(res, indent=1, default=float))
    for m in ("base", "ley", "memoria"):
        l = [r for r in res if r["modo"] == m]
        print(f"{m:8s} éxito {100*sum(r['exito'] for r in l)/len(l):5.1f}%  contactos/ep {sum(r['contactos'] for r in l)/len(l):.2f}  "
              f"rmse {sum(r['rmse_cm'] for r in l)/len(l):5.1f} cm  pico {sum(r['pico_cm'] for r in l)/len(l):5.1f} cm  progreso {sum(r['progreso'] for r in l)/len(l):.2f}", flush=True)
    print("total", round(time.time() - t0), "s")


if __name__ == "__main__":
    main()
