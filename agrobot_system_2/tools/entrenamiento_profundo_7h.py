#!/usr/bin/env python3
"""ENTRENAMIENTO PROFUNDO de ~7 h (2026-10-28). Fases en SERIE (la Jetson no aguanta dos entrenamientos a la vez: OOM), cada una con su tope de tiempo; si una falla, sigue la siguiente.
No despliega nada: todo queda como candidato en ~/.agrobot/aprendizaje/profundo_<fecha>/ y en modelos/. Uso: nohup python3 tools/entrenamiento_profundo_7h.py [--horas 7] &
Fases: 1 verificar datos · 2 Monte Carlo/CEM en el L · 3 CEM general interior/exterior · 4 visión (YOLO objetos y plagas, transitabilidad con fotos reales) · 5 política por imitación · 6 resumen."""
import argparse, json, os, shutil, subprocess, sys, time
from pathlib import Path

AQUI = Path(__file__).resolve().parent.parent
SP = AQUI / "simulador_pasivo"
PS = SP / "percepcion_sintetica"
ENV = dict(os.environ, PYTHONPATH=str(Path.home() / ".agrobot" / "numba_pkg") + ":" + os.environ.get("PYTHONPATH", ""))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--horas", type=float, default=7.0); a = ap.parse_args()
    t_fin = time.time() + a.horas * 3600
    sal = Path.home() / ".agrobot" / "aprendizaje" / ("profundo_" + time.strftime("%Y%m%d_%H%M")); sal.mkdir(parents=True, exist_ok=True)
    log = open(sal / "log.txt", "a"); estado = {}

    def say(*x):
        s = time.strftime("%H:%M:%S ") + " ".join(str(i) for i in x); print(s, flush=True); log.write(s + "\n"); log.flush()

    def correr(nombre, cmd, cwd, tope_min, env=ENV):
        t0 = time.time(); resto = (t_fin - t0) / 60
        tope = max(1.0, min(tope_min, resto - 5))
        say(f"== {nombre} (tope {tope:.0f} min): {' '.join(map(str, cmd))}")
        try:
            with open(sal / f"{nombre}.log", "w") as f:
                r = subprocess.run(cmd, cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT, timeout=tope * 60)
            estado[nombre] = {"rc": r.returncode, "min": round((time.time() - t0) / 60, 1)}
        except subprocess.TimeoutExpired:
            estado[nombre] = {"rc": "tope de tiempo", "min": round((time.time() - t0) / 60, 1)}
        except Exception as e:                                              # noqa: BLE001
            estado[nombre] = {"rc": repr(e), "min": 0}
        say(f"   -> {estado[nombre]}")
        (sal / "estado.json").write_text(json.dumps(estado, indent=1, ensure_ascii=False))

    py = sys.executable
    # 1. verificación de datos
    correr("1_datos", [py, str(AQUI / "tools" / "verificar_datos.py")], AQUI, 15)
    # 2. Monte Carlo / CEM en el L (el escenario de las pruebas), 140 min
    correr("2_L_cem", [py, "-u", "entrenar_L_cem.py", "--horas", "2.3", "--final-min", "18", "--procs", "5"], SP, 150)
    # 3. CEM general (interior 70 % / exterior 30 %), 70 min
    correr("3_cem_general", [py, "-u", "entrenar_intensivo.py", "--horas", "1.15", "--final-min", "12", "--procs", "5"], SP, 75)
    # 4. visión: respaldo de pesos actuales y entrenamientos (solo candidatos)
    resp = PS / "modelos" / "yolo_respaldo_20261028"
    if not resp.exists():
        shutil.copytree(PS / "modelos" / "yolo", resp)
    correr("4a_yolo_obj", [py, "-u", "entrenar_yolo.py", "--tarea", "obj", "--epocas", "40"], PS, 35)
    correr("4b_yolo_plagas", [py, "-u", "entrenar_yolo.py", "--tarea", "plagas", "--epocas", "40"], PS, 40)
    correr("4c_transitabilidad_real", [py, "-u", "entrenar_trav.py", "--real", "--epocas", "12", "--nombre", "trav_20261028"], PS, 30)
    # 5. política por imitación con lo que quede (menos 15 min para el resumen)
    resto = (t_fin - time.time()) / 3600 - 0.25
    if resto > 0.2:
        fin = int(time.time() + resto * 3600)
        env = dict(os.environ, AGROBOT_INICIO_ENTRENAMIENTO=time.strftime("%Y%m%d_%H%M"))
        correr("5_politica", ["bash", str(AQUI / "tools" / "supervisor_entrenamiento.sh"), str(fin), str(sal / "5_politica_entrenar.log"),
                              "--mundos", "bucle,hileras,pasillo,pasillo_real,callejon,trampa_bucle,invernadero,serpentina,campo_abierto,exterior_2_sim,huerta_real"], AQUI, resto * 60 + 2, env)
    say("== fin de las fases"); say(json.dumps(estado, ensure_ascii=False))
    (sal / "estado.json").write_text(json.dumps({"fases": estado, "terminado": time.strftime("%Y-%m-%d %H:%M:%S")}, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
