"""Verifica la config de CONSENSO (sacada de la sensibilidad del Montecarlo) contra la elegida y las referencias."""
import json, time
from multiprocessing import Pool
import montecarlo_firmware_pesado as M

d = json.load(open(sorted(M.SALIDA.glob("mc_pesado_2*.json"))[-1]))
CONSENSO = dict(firmware="fusion", fu_adel=80, fu_atras=60, fu_pivote=100, fu_minp=15, fu_acel=100, fu_frena=400,
                fu_tras=150, fu_dif_del=0, fu_periodo=100, fu_refuerzo=100,
                cc_v=0.18, cc_kp=1.5, cc_kd=0.6, cc_kh=2.5, cc_kl=0.8, cc_wmax=0.6, cc_dstop=0.42)
cands = {"CONSENSO": CONSENSO, "ELEGIDO_MC": d["elegido"]["config"], "SUAVE_PEGADO": M.REF["SUAVE_PEGADO"],
         "SUAVE_REPO": M.REF["SUAVE_REPO"]}
t = time.time()
with Pool(6) as p:
    rs = M.evaluar(p, list(cands.values()), list(range(600, 616)))
out = {k: {"total": M.resumen(r), "por_escenario": M.por_esc(r)} for k, r in zip(cands, rs)}
out["config_consenso"] = CONSENSO
json.dump(out, open(M.SALIDA / "mc_consenso.json", "w"), indent=1)
for k in cands: print(k, out[k]["total"])
for e, v in out["CONSENSO"]["por_escenario"].items(): print("  CONSENSO", e, v["exito_pct"], v["contactos_por_ep"])
print(round(time.time() - t), "s")
