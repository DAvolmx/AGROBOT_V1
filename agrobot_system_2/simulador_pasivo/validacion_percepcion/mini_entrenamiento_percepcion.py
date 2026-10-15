#!/usr/bin/env python3
"""Mini-experimento: conducir el huerto CON la percepción del modelo vs SIN ella.

El robot rústico sigue el sendero curvo de `huerta_fotos_50cm`:
 · SIN modelo: no ve el sendero (la cámara no le da señal de sendero); mantiene
   el rumbo inicial con odometría -> no puede seguir la curva y se sale.
 · CON modelo: cada ~0.3 s la UNet segmenta el fotograma, se saca el CENTRO del
   sendero en el campo cercano y se dirige hacia él -> sigue la curva.

Se mide el error de seguimiento al sendero (VERDAD del sim, solo para puntuar;
nunca entra al controlador). Genera un PDF de reporte.

    python3 mini_entrenamiento_percepcion.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import cv2
import torch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parent
sys.path.insert(0, str(AQUI)); sys.path.insert(0, str(RAIZ))
sys.path.insert(0, str(RAIZ.parent / "jetson" / "backend"))

import mundo as mundo_mod                       # noqa: E402
from dataclasses import replace                 # noqa: E402
from dinamica_pasiva import DinamicaPasiva       # noqa: E402
from robot_rustico import parametros_rusticos    # noqa: E402
import validar_sim as V                          # noqa: E402

MID = 0.185
PDF = AQUI / "reporte_percepcion.pdf"


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _proyecta(pts, p):
    best_d, best_s, acc = 1e9, 0.0, 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        a = np.asarray(a); b = np.asarray(b); ab = b - a
        L = np.hypot(*ab)
        if L < 1e-9:
            continue
        t = max(0.0, min(1.0, np.dot(np.asarray(p) - a, ab) / (L * L)))
        d = float(np.hypot(*(np.asarray(p) - (a + t * ab))))
        if d < best_d:
            best_d, best_s = d, acc + t * L
        acc += L
    return best_d, best_s


def sendero_offset(pred):
    """Centro del sendero (clase 1) en el campo cercano -> offset [-1,1].
    >0 = sendero a la derecha. None si casi no ve sendero."""
    h, w = pred.shape
    banda = pred[int(h * 0.45):, :]
    cols = np.where(banda == 1)[1]
    if len(cols) < banda.size * 0.02:
        return None
    return float((cols.mean() - w / 2) / (w / 2))


def _adelante(pts, s0, la=0.5):
    acc = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        L = math.dist(a, b)
        if acc + L >= s0 + la:
            return b
        acc += L
    return pts[-1]


def conducir(modo, semilla, modelo, dev, mundo, geo, sendero,
             dt=0.08, t_max=20.0, refresh=4, v_base=0.4):
    pts = np.array(sendero, float)
    largo = sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
    p = replace(parametros_rusticos(semilla), mu=0.6, substep=0.01)
    psi0 = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    d = DinamicaPasiva(pose=(pts[0][0] - MID * math.cos(psi0),
                             pts[0][1] - MID * math.sin(psi0), psi0), parametros=p)
    rng = np.random.default_rng(semilla)
    errs, on, traj, s_max = [], 0, [], 0.0
    offset, muestra = 0.0, None
    for k in range(int(t_max / dt)):
        x, y, psi, delta = d.q
        xm, ym = x + MID * math.cos(psi), y + MID * math.sin(psi)
        dcc, s = _proyecta(pts, (xm, ym))
        errs.append(dcc); s_max = max(s_max, s); traj.append((xm, ym))
        if dcc < 0.25:
            on += 1
        if dcc > 1.2:
            break                                   # perdido
        if modo == "con_modelo":
            if k % refresh == 0:
                bgr = mundo.camara(xm - MID * math.cos(psi), ym - MID * math.sin(psi),
                                   psi, geo, 320, 240)
                pred = V.inferir(modelo, dev, bgr)
                o = sendero_offset(pred)
                if o is not None:
                    offset = o
                if muestra is None and k > 6:
                    muestra = (bgr.copy(), V.superponer(bgr, pred))
            omega = float(np.clip(-1.4 * offset, -1.2, 1.2))
            v = v_base / (1 + 1.2 * abs(offset))
        elif modo == "oraculo":   # percepción PERFECTA: sigue el sendero real (cota superior)
            obj = _adelante(pts, s, 0.5)
            e = _wrap(math.atan2(obj[1] - ym, obj[0] - xm) - psi)
            omega = float(np.clip(2.0 * e, -1.2, 1.2))
            v = v_base / (1 + 1.0 * abs(e))
        else:   # sin_modelo: mantiene el rumbo inicial con odometría (sin visión)
            psie = psi + rng.normal(0, math.radians(1.0))
            omega = float(np.clip(1.5 * _wrap(psi0 - psie), -1.2, 1.2))
            v = v_base
        d.step(dt, v, omega)
    n = len(errs)
    return {"rmse_cm": round(float(np.sqrt(np.mean(np.square(errs)))) * 100, 1),
            "max_cm": round(max(errs) * 100, 1),
            "en_sendero_pct": round(100 * on / n, 1),
            "progreso": round(min(1.0, s_max / largo), 3),
            "traj": np.array(traj), "muestra": muestra}


def main():
    print("Cargando modelo...", flush=True)
    modelo, dev = V.cargar_modelo()
    d = json.load(open(RAIZ / "mundos" / "huerta_fotos_50cm.json"))
    mundo = mundo_mod.Mundo.desde_dict(d)
    cfg = json.load(open(RAIZ / "config_huerta.json"))
    geo = dict(cfg["odometria_webcam"]); geo["camara_inclinacion_deg"] = 33.0
    sendero = d["sendero"]
    semillas = [0, 1]
    MODOS = ("sin_modelo", "con_modelo", "oraculo")

    res = {m: [] for m in MODOS}
    ejemplo = {}
    for modo in MODOS:
        for s in semillas:
            m = conducir(modo, s, modelo, dev, mundo, geo, sendero)
            res[modo].append(m)
            print(f"  {modo} seed {s}: rmse {m['rmse_cm']}cm, "
                  f"en_sendero {m['en_sendero_pct']}%, progreso {m['progreso']}", flush=True)
        ejemplo[modo] = res[modo][0]

    def prom(modo, k):
        return round(float(np.mean([e[k] for e in res[modo]])), 1)

    resumen = {m: {"rmse_cm": prom(m, "rmse_cm"), "en_sendero_pct": prom(m, "en_sendero_pct"),
                   "progreso": round(float(np.mean([e["progreso"] for e in res[m]])), 2)}
               for m in ("sin_modelo", "con_modelo", "oraculo")}
    print("\nRESUMEN:", json.dumps(resumen, ensure_ascii=False))
    _pdf(res, resumen, sendero, ejemplo)
    print(f"\nPDF -> {PDF}")


def _pdf(res, resumen, sendero, ejemplo):
    pts = np.array(sendero, float)
    with PdfPages(PDF) as pdf:
        # --- Página 1: portada + tabla ---
        fig = plt.figure(figsize=(8.3, 11.7))
        fig.text(0.5, 0.95, "AgroBot · conducción en simulación", ha="center", size=16, weight="bold")
        fig.text(0.5, 0.915, "Rendimiento CON la percepción del modelo vs SIN ella", ha="center", size=11)
        fig.text(0.08, 0.86, "2026-10-11 · simulador (huerta_fotos_50cm) · chasis rústico, 3 semillas", size=9, color="gray")
        filas = [["modo", "error RMS (cm)", "en sendero (%)", "progreso"]]
        for m in ("sin_modelo", "con_modelo", "oraculo"):
            r = resumen[m]
            filas.append([m, r["rmse_cm"], r["en_sendero_pct"], r["progreso"]])
        ax = fig.add_axes([0.08, 0.62, 0.84, 0.18]); ax.axis("off")
        t = ax.table(cellText=filas[1:], colLabels=filas[0], loc="center", cellLoc="center")
        t.scale(1, 2); t.set_fontsize(11)
        texto = (
            "Qué se midió\n"
            "  El robot sigue el sendero curvo del huerto. SIN modelo no ve el sendero\n"
            "  (mantiene el rumbo con odometría) y no puede seguir la curva. CON modelo,\n"
            "  la UNet segmenta la cámara cada ~0.3 s, se toma el centro del sendero y se\n"
            "  dirige hacia él. El error de seguimiento es VERDAD del sim, solo para puntuar.\n\n"
            "  'oraculo' = percepcion PERFECTA (sigue el sendero real): cota superior de\n"
            "  lo que daria una buena percepcion, y prueba que el sim/control SI pueden\n"
            "  seguir el sendero. La diferencia con 'con_modelo' es la brecha de percepcion.\n\n"
            "Avisos de honestidad\n"
            "  · Preprocesado de 6 canales RECONSTRUIDO (no el exacto de Codex) -> la\n"
            "    inferencia es cualitativa; con su preprocesado mejoraria.\n"
            "  · Brecha de dominio: modelo entrenado con fotos, evaluado en sim sintetico\n"
            "    (Codex: mIoU_real 0.73 vs mIoU_synth 0.45). En estas vistas el modelo\n"
            "    predice MUY POCO sendero (2-4% px) -> no da senal de direccion fiable.\n"
            "  · La clase obstaculo_visible no se predice (su dataset no la tenia)."
        )
        fig.text(0.08, 0.30, texto, size=9.5, family="monospace", va="top")
        pdf.savefig(fig); plt.close(fig)

        # --- Página 2: trayectorias ---
        fig, ax = plt.subplots(figsize=(8.3, 6))
        ax.plot(pts[:, 0], pts[:, 1], "k--", lw=1, label="sendero (verdad)")
        for modo, col in (("sin_modelo", "#d9534f"), ("con_modelo", "#2a78d6"), ("oraculo", "#2aa84a")):
            tr = res[modo][0]["traj"]
            ax.plot(tr[:, 0], tr[:, 1], color=col, lw=2, label=modo)
        ax.set_aspect("equal"); ax.legend(); ax.set_title("Trayectoria (semilla 0)")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=.3)
        pdf.savefig(fig); plt.close(fig)

        # --- Página 3: barras comparativas ---
        fig, axes = plt.subplots(1, 2, figsize=(8.3, 4))
        modos = ["sin_modelo", "con_modelo"]
        axes[0].bar(modos, [resumen[m]["rmse_cm"] for m in modos], color=["#d9534f", "#2a78d6"])
        axes[0].set_title("Error de seguimiento RMS (cm)  menos=mejor")
        axes[1].bar(modos, [resumen[m]["en_sendero_pct"] for m in modos], color=["#d9534f", "#2a78d6"])
        axes[1].set_title("% del tiempo en el sendero  más=mejor"); axes[1].set_ylim(0, 100)
        fig.tight_layout(); pdf.savefig(fig); plt.close(fig)

        # --- Página 4: ejemplo cámara + segmentación ---
        mu = ejemplo["con_modelo"]["muestra"]
        if mu is not None:
            rgb, over = mu
            fig, axes = plt.subplots(1, 2, figsize=(8.3, 3.5))
            axes[0].imshow(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)); axes[0].set_title("cámara del sim"); axes[0].axis("off")
            axes[1].imshow(cv2.cvtColor(over, cv2.COLOR_BGR2RGB)); axes[1].set_title("segmentación del modelo"); axes[1].axis("off")
            fig.suptitle("Lo que ve el modelo durante la conducción (CON modelo)")
            pdf.savefig(fig); plt.close(fig)


if __name__ == "__main__":
    main()
