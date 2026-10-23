"""Simulación de la config que quedó FLASHEADA el 2026-10-23 (fusión, minp 100 = 100 % continuo, dif_del 100) con el
hardware corregido (delanteras FUERTES, traseras flojas, robot pesado). Compara contra v3, suave pegado y cand1, y
barre cada parámetro del firmware alrededor de la config de hoy. Salida: ~/.agrobot/aprendizaje/sim_hoy_<fecha>.json"""
import json, time
from multiprocessing import Pool
import montecarlo_firmware_pesado as M

HOY = dict(firmware="fusion", fu_adel=100, fu_atras=100, fu_pivote=100, fu_minp=100, fu_acel=600, fu_frena=300,
           fu_tras=150, fu_dif_del=100, fu_periodo=0, fu_refuerzo=100, **M.CC_MEDIO)
HOY_RAFAGA = dict(HOY, fu_minp=80, fu_periodo=50, fu_dif_del=0)          # lo que probó el operador antes (solo avanzaba)
CAND1 = dict(firmware="fusion", fu_adel=71.09, fu_atras=62.969, fu_pivote=96.316, fu_minp=15.401, fu_acel=116.225,
             fu_frena=294.397, fu_tras=100.0, fu_dif_del=69.427, fu_periodo=165.06, fu_refuerzo=114.396, **M.CC_MEDIO)
REFS = {"HOY(flasheada)": HOY, "ANTES(rafaga50,dif0)": HOY_RAFAGA, "CAND1": CAND1,
        "V3": M.REF["V3"], "SUAVE_PEGADO": M.REF["SUAVE_PEGADO"]}


def main():
    t0 = time.time()
    out = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "hardware": "delanteras fuertes (bias 1.5-3.5), masa 14-24 kg"}
    with Pool(4) as pool:
        rs = M.evaluar(pool, list(REFS.values()), list(range(800, 816)))
        out["comparacion"] = {k: {"total": M.resumen(r), "por_escenario": M.por_esc(r)} for k, r in zip(REFS, rs)}
        for k, v in out["comparacion"].items():
            print(k, v["total"], flush=True)
        json.dump(out, open(M.SALIDA / "sim_hoy_parcial.json", "w"), indent=1)
        print("== barrido de un parámetro cada vez alrededor de la config de hoy ==", flush=True)
        niv = {k: v for k, v in M.RANGOS.items() if k.startswith("fu_") and k != "fu_refuerzo"}
        out["barrido"] = {}
        M.RANGOS = niv
        out["barrido"] = M.barrido_uno(pool, HOY, list(range(900, 904)))
        for k, v in out["barrido"].items():
            print(k, [(x["valor"], x["puntos"], x["exito_pct"]) for x in v], flush=True)
        print("== estrés (24 kg, traseras al mínimo, zona muerta 55 %) ==", flush=True)
        rs = M.evaluar(pool, [HOY, M.REF["V3"], M.REF["SUAVE_PEGADO"]], list(range(1100, 1108)), extremo=True)
        out["estres"] = {n: {"total": M.resumen(r), "por_escenario": M.por_esc(r)} for n, r in zip(["HOY", "V3", "SUAVE_PEGADO"], rs)}
        for k, v in out["estres"].items():
            print("estrés", k, v["total"], flush=True)
    out["duracion_min"] = round((time.time() - t0) / 60)
    path = M.SALIDA / f"sim_hoy_{time.strftime('%Y%m%d_%H%M')}.json"
    json.dump(out, open(path, "w"), indent=1)
    print("Guardado", path, flush=True)


if __name__ == "__main__":
    main()
