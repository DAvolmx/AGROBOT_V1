"""Análisis de la prueba de EXTERIOR del 2026-10-27 (misión de plantas en maceta con mando manual intercalado): saca métricas y figuras para el reporte de la tesis.
Lee ~/.agrobot/datos/20261027_215547 (eventos, telemetría, scans), las sesiones de campo 20261027_215828_25f568 y 20261027_221926_28a5d4 (videos),
las fotos de la visión activa y las muestras del clasificador de entorno. No modifica nada de lo que lee.
Uso: python3 tools/analizar_prueba_exterior_20261027.py   Salida: docs/PRUEBA_EXTERIOR_20261027/{figs/*.png, metricas.json}"""
import csv, datetime, glob, json, math, os, re
from collections import Counter
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RAIZ = Path(__file__).resolve().parents[1]
OUT = RAIZ / "docs" / "PRUEBA_EXTERIOR_20261027"
FIG = OUT / "figs"
HOME = Path.home() / ".agrobot"
DATOS = HOME / "datos" / "20261027_215547"
DATOS_INTERIOR = HOME / "datos" / "20261027_191114"             # sesión de interior del mismo día (para comparar el LIDAR)
CAMPOS = [HOME / "campo" / "20261027_215828_25f568", HOME / "campo" / "20261027_221926_28a5d4"]
VENTANA = (150.0, 880.0)       # s de sesión (reloj MONOTÓNICO): del primer movimiento al fin de la misión. El reloj de pared saltó +711,5 s en t=527 (sin RTC): NO se usa t_unix
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.25, "figure.dpi": 130, "axes.spines.top": False, "axes.spines.right": False})
VERDE, AZUL, NAR, ROJO, GRIS = "#1f9d55", "#2b6cb0", "#dd6b20", "#c53030", "#718096"


def jsonl(p):
    out = []
    for l in open(p, encoding="utf-8", errors="replace"):
        if l.startswith("{"):
            try:
                out.append(json.loads(l))
            except ValueError:
                pass
    return out


def categoria(estado):
    e = str(estado or "")
    if e.startswith("en pausa"): return "mando manual"
    if e.startswith("avanzando"): return "avanzando"
    if e.startswith("girando"): return "girando"
    if e.startswith("esquina") or e.startswith("algo a"): return "maniobra de esquina"
    if e.startswith("sendero") or e.startswith("guardián"): return "corrigiendo rumbo"
    if e.startswith("escaneando"): return "escaneando"
    if e.startswith("detenido"): return "detenido"
    return "otro"


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    R = jsonl(DATOS / "eventos.jsonl")
    T0 = R[0]["t_unix"] - R[0]["t"]                               # t (s de sesión) -> unix, ANTES del salto del reloj
    te_, ue_ = np.array([r["t"] for r in R]), np.array([r["t_unix"] for r in R])
    salto = np.where(np.abs(np.diff(ue_ - te_)) > 1)[0]
    T_SALTO, D_SALTO = (float(te_[salto[0]]), float((ue_ - te_)[salto[0] + 1] - (ue_ - te_)[salto[0]])) if len(salto) else (None, 0.0)

    def mono(u):                                                 # reloj de pared -> s de sesión (deshace el salto)
        u = np.asarray(u, float)
        return np.where(u < T0 + (T_SALTO or 1e18) + 1, u - T0, u - T0 - D_SALTO)
    tel = list(csv.DictReader(open(DATOS / "telemetria.csv")))
    num = lambda k, d=float("nan"): np.array([float(r[k]) if r.get(k) not in ("", None) else d for r in tel])
    tu, va, wa = num("t"), num("v_aplicada"), num("w_aplicada")
    gz = num("giro_z")
    mov = (np.abs(va) > 0.01) | (np.abs(wa) > 0.05)
    en_ventana = (tu >= VENTANA[0]) & (tu <= VENTANA[1])
    M = {"sesion": "20261027_215547", "entorno": "exterior", "ventana_s_sesion": list(VENTANA), "salto_reloj": {"en_t_s": T_SALTO, "magnitud_s": round(D_SALTO, 1)}}

    # ---------------- misión
    ev_m = [(r["t"], r["datos"]) for r in R if r["tipo"] == "mision_plantas"]
    ini = next(t for t, d in ev_m if d.get("tipo") == "mision_inicio")
    fin = next(t for t, d in ev_m if d.get("tipo") == "mision_fin")
    plantas = [d for _, d in ev_m if d.get("tipo") == "planta_inspeccionada"]
    fallidas = [d.get("motivo") for _, d in ev_m if d.get("tipo") == "inspeccion_fallida"]
    M["mision"] = {"duracion_s": round(fin - ini), "plantas_inspeccionadas": len(plantas), "inspecciones_fallidas": len(fallidas), "motivos_fallo": dict(Counter(fallidas)),
                   "plantas": [{k: p.get(k) for k in ("n", "x", "y", "especie", "p_col", "vistas")} for p in plantas],
                   "detecciones_de_planta": sum(1 for _, d in ev_m if d.get("tipo") == "planta_detectada")}

    # ---------------- exploración: estados
    est = [(r["t"], categoria(r["datos"].get("estado")), r["datos"]) for r in R if r["tipo"] == "exploracion"]
    cnt = Counter(c for t, c, _ in est if VENTANA[0] <= t <= VENTANA[1])
    tot = sum(cnt.values()) or 1
    M["estados_pct"] = {k: round(100 * v / tot, 1) for k, v in cnt.most_common()}
    M["manual_pct_de_activo"] = round(100 * cnt.get("mando manual", 0) / max(1, tot - cnt.get("detenido", 0)), 1)

    # ---------------- recorrido y distancias
    dt = np.gradient(tu)
    # picos del EKF (un cuadro con |vx| absurdo): se descartan del cálculo de camino
    vxe = num("vx_ekf"); xe, ye = num("x"), num("y")
    pico = np.abs(vxe) > 3.0
    M["ekf"] = {"cuadros_con_vx_absurda": int(pico.sum()), "vx_max_abs_ms": round(float(np.nanmax(np.abs(vxe))), 2), "x_max_abs_m": round(float(np.nanmax(np.abs(xe))), 1),
                "salto_de_pose_tras_el_salto_de_reloj_m": None}
    if T_SALTO is not None:
        a, b = np.argmin(np.abs(tu - (T_SALTO - 0.5))), np.argmin(np.abs(tu - (T_SALTO + 2.0)))
        M["ekf"]["salto_de_pose_tras_el_salto_de_reloj_m"] = round(float(np.hypot(xe[b] - xe[a], ye[b] - ye[a])), 1)
    camino = float(np.nansum(np.abs(va[en_ventana & mov & ~pico]) * dt[en_ventana & mov & ~pico]))
    pos = [(r["t"], r["datos"]["pose"]) for r in R if r["tipo"] == "scan_mapa" and r["datos"].get("pose")]
    px = np.array([p["x"] for _, p in pos]); py = np.array([p["y"] for _, p in pos]); pt = np.array([t for t, _ in pos]); pth = np.array([p["theta_deg"] for _, p in pos])
    sel = (pt >= VENTANA[0]) & (pt <= VENTANA[1])
    d_slam = np.hypot(np.diff(px[sel]), np.diff(py[sel])) if sel.sum() > 2 else np.zeros(0)
    saltos_slam = [(round(float(pt[sel][i + 1]), 1), round(float(d_slam[i]), 1)) for i in np.where(d_slam > 0.5)[0]]
    camino_slam = float(d_slam[d_slam <= 0.5].sum())
    M["recorrido"] = {"camino_slam_m": round(camino_slam, 1), "camino_integral_v_m": round(camino, 1), "saltos_de_pose_slam": saltos_slam, "tiempo_en_movimiento_s": round(float(np.sum(dt[en_ventana & mov & (dt < 1)])), 1),
                      "extension_xy_m": [round(float(np.ptp(px[sel])), 1), round(float(np.ptp(py[sel])), 1)] if sel.sum() else None}
    lid = {k: num("lidar_" + k) for k in ("adelante", "atras", "izquierda", "derecha")}
    cer = {k: float(np.nanmin(v[en_ventana])) for k, v in lid.items()}
    M["lidar_min_m"] = {k: round(v, 2) for k, v in cer.items()}
    M["lidar_pct_menos_20cm"] = {k: round(100 * float(np.nanmean(v[en_ventana & (v > 0.01)] < 0.20)), 1) for k, v in lid.items()}
    M["lidar_p5_m"] = {k: round(float(np.nanpercentile(v[en_ventana & (v > 0.01)], 5)), 2) for k, v in lid.items()}
    M["intervenciones_manuales"] = {"eventos_asistido": sum(1 for r in R if r["tipo"] == "asistido" and VENTANA[0] <= r["t"] <= VENTANA[1]),
                                    "toasts_autonomia_reanudada": sum(1 for r in R if r["tipo"] == "toast" and "reanudada" in str(r["datos"].get("title")) and VENTANA[0] <= r["t"] <= VENTANA[1])}
    M["seguridad"] = dict(Counter(r["datos"].get("nivel") for r in R if r["tipo"] == "seguridad"))
    M["anticolision"] = sum(1 for r in R if r["tipo"] == "toast" and "Anticolisi" in str(r["datos"].get("title")))
    M["avisos_plaga_toast"] = Counter(str(r["datos"].get("message")).split(" · ")[0] for r in R if r["tipo"] == "toast" and "Plaga" in str(r["datos"].get("title")))
    M["avisos_plaga_toast"] = dict(M["avisos_plaga_toast"])
    M["capturas_plaga"] = sum(1 for r in R if r["tipo"] == "capture")

    # ---------------- F1: trayectoria + mapa de puntos
    fig, ax = plt.subplots(figsize=(7.2, 6.2))
    pts = []
    for r in R:
        if r["tipo"] == "scan_mapa" and VENTANA[0] <= r["t"] <= VENTANA[1] and r["datos"].get("pts"):
            a = np.array(r["datos"]["pts"], float).reshape(-1, 2) / 100.0
            pts.append(a[::2])
    if pts:
        P = np.vstack(pts)
        ax.hist2d(P[:, 0], P[:, 1], bins=[int(np.ptp(P[:, 0]) / 0.05) + 1, int(np.ptp(P[:, 1]) / 0.05) + 1], cmap="Greys", cmin=3, norm=matplotlib.colors.LogNorm(), zorder=1)
    if sel.sum():
        cx_, cy_ = np.median(px[sel]), np.median(py[sel])
        bueno = sel & (np.hypot(px - cx_, py - cy_) < 15.0)           # los cuadros con la pose saltada (marco corrupto) se dibujan aparte, no aplastan el recorrido
        M["recorrido"]["poses_slam_descartadas_por_salto"] = int((sel & ~bueno).sum())
        sc = ax.scatter(px[bueno], py[bueno], c=(pt[bueno] - VENTANA[0]) / 60.0, s=3, cmap="viridis", zorder=3)
        plt.colorbar(sc, ax=ax, label="minutos desde el inicio de la prueba", shrink=0.8)
        ax.scatter(px[bueno][0], py[bueno][0], marker="s", s=70, c="white", edgecolors="k", zorder=5, label="inicio")
        ax.scatter(px[bueno][-1], py[bueno][-1], marker="*", s=130, c=NAR, edgecolors="k", zorder=5, label="fin")
        for ts_, dj in saltos_slam:
            i = int(np.argmin(np.abs(pt - ts_))) - 1
            ax.scatter(px[i], py[i], marker="x", s=70, c=ROJO, zorder=8)
        if saltos_slam:
            ax.text(0.02, 0.02, "× rojas: la pose del SLAM saltó " + ", ".join(f"{d:.0f} m (t={t:.0f} s)" for t, d in saltos_slam if d > 5) + "\n(esos cuadros se omiten del dibujo)", transform=ax.transAxes, fontsize=7, color=ROJO, va="bottom")
    for p in plantas:
        if p.get("x") is not None:
            ax.scatter(p["x"], p["y"], marker="D", s=60, c=VERDE, edgecolors="k", zorder=6)
            ax.annotate(f"P{p['n']}", (p["x"], p["y"]), textcoords="offset points", xytext=(6, 6), fontsize=8, weight="bold", color=VERDE, zorder=7)
    ax.set_aspect("equal"); ax.set_xlabel("x (m, marco del mapa SLAM)"); ax.set_ylabel("y (m)")
    ax.set_title("Prueba de exterior: recorrido (pose del SLAM), puntos LIDAR acumulados y plantas escaneadas", fontsize=9); ax.legend(loc="best", fontsize=8)
    fig.tight_layout(); fig.savefig(FIG / "f1_trayectoria.png"); plt.close(fig)

    # ---------------- F2: línea de tiempo
    cats = ["avanzando", "girando", "maniobra de esquina", "corrigiendo rumbo", "escaneando", "mando manual", "detenido", "otro"]
    col = dict(zip(cats, [VERDE, AZUL, NAR, "#805ad5", "#d69e2e", ROJO, "#cbd5e0", GRIS]))
    t_ini, t_fin = VENTANA
    fig, axs = plt.subplots(3, 1, figsize=(9.5, 6.4), sharex=True, gridspec_kw={"height_ratios": [1.1, 1.3, 1.6]})
    ax = axs[0]
    for i, (t, c, _) in enumerate(est):
        if t_ini <= t <= t_fin:
            ax.axvspan((t - t_ini) / 60, (t + 2) / 60 - t_ini / 60, color=col[c], lw=0)
    for t, d in ev_m:
        if d.get("tipo") in ("planta_inspeccionada", "inspeccion_fallida", "mision_inicio", "mision_fin") and t_ini <= t <= t_fin:
            ax.axvline((t - t_ini) / 60, color="k", lw=1.2 if d["tipo"] != "inspeccion_fallida" else 0.6, ls="-" if d["tipo"] == "planta_inspeccionada" else ":")
    ax.set_yticks([]); ax.set_title("Línea de tiempo de la prueba (líneas continuas: planta escaneada; punteadas: inspección fallida)")
    ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=col[c]) for c in cats[:7]], labels=cats[:7], ncol=7, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.02), frameon=False)
    ax = axs[1]
    m = en_ventana
    ax.plot((tu[m] - t_ini) / 60, np.abs(va[m]), color=VERDE, lw=0.8, label="|v| aplicada (m/s)")
    ax.plot((tu[m] - t_ini) / 60, np.abs(wa[m]), color=AZUL, lw=0.8, label="|ω| aplicada (rad/s)")
    ax.legend(fontsize=7, loc="upper right"); ax.set_ylabel("mando")
    ax = axs[2]
    for k, c in zip(("adelante", "izquierda", "derecha", "atras"), (VERDE, AZUL, NAR, GRIS)):
        ax.plot((tu[m] - t_ini) / 60, np.clip(lid[k][m], 0, 3), color=c, lw=0.8, label=k)
    ax.axhline(0.2, color=ROJO, ls="--", lw=0.8); ax.text(0.2, 0.22, "20 cm", color=ROJO, fontsize=7)
    ax.set_ylabel("hueco libre LIDAR (m, tope 3)"); ax.set_xlabel("minutos"); ax.legend(fontsize=7, ncol=4, loc="upper right")
    fig.tight_layout(); fig.savefig(FIG / "f2_linea_tiempo.png"); plt.close(fig)

    # ---------------- F3: pulsos de giro (telemetría). Dos estimaciones independientes del ángulo por pulso: integral del giroscopio y cambio del rumbo del EKF.
    rum = num("rumbo_deg")
    pulsos = []
    i, n = 0, len(tu)
    while i < n:
        if en_ventana[i] and abs(wa[i]) > 0.1 and abs(va[i]) < 0.05:
            j = i
            while j + 1 < n and abs(wa[j + 1]) > 0.1 and tu[j + 1] - tu[j] < 0.6:
                j += 1
            k = min(n - 1, j + 4)
            sg = float(np.sign(wa[i]))
            ang_g = sg * float(np.nansum(gz[i:k + 1] * dt[i:k + 1]))
            ang_r = sg * float((rum[k] - rum[max(i - 1, 0)] + 180) % 360 - 180)
            pulsos.append((tu[i], tu[j] - tu[i] + 0.15, ang_g, ang_r))
            i = j + 1
        else:
            i += 1
    P3 = np.array(pulsos) if pulsos else np.zeros((0, 4))
    if len(P3):
        ag, ar = P3[:, 2], P3[:, 3]
        resumen = lambda a: {"mediana_deg": round(float(np.median(a)), 1), "p25_deg": round(float(np.percentile(a, 25)), 1), "p75_deg": round(float(np.percentile(a, 75)), 1),
                             "menos_de_2deg_pct": round(100 * float(np.mean(np.abs(a) < 2))), "sentido_contrario_pct": round(100 * float(np.mean(a < -1)))}
        M["giros"] = {"pulsos": int(len(P3)), "objetivo_deg": 6.0, "giroscopio": resumen(ag), "rumbo_ekf": resumen(ar), "duracion_mediana_s": round(float(np.median(P3[:, 1])), 2)}
        fig, axs = plt.subplots(1, 2, figsize=(9.5, 3.6))
        bins = np.arange(-8, 20, 1.0)
        axs[0].hist(ag, bins=bins, alpha=0.65, color=AZUL, label="integral del giroscopio")
        axs[0].hist(ar, bins=bins, alpha=0.65, color=VERDE, label="cambio del rumbo del EKF")
        axs[0].axvline(6, color=ROJO, ls="--"); axs[0].text(6.2, axs[0].get_ylim()[1] * 0.9, "objetivo 6°", color=ROJO, fontsize=8)
        axs[0].set_xlabel("ángulo girado por pulso (°, + = en el sentido pedido)"); axs[0].set_ylabel(f"pulsos (n={len(P3)})"); axs[0].legend(fontsize=7)
        axs[1].scatter(P3[:, 1], ag, s=8, c=AZUL, alpha=0.7)
        axs[1].axhline(6, color=ROJO, ls="--", lw=0.8); axs[1].axhline(0, color="k", lw=0.6)
        axs[1].set_xlabel("duración del pulso (s)"); axs[1].set_ylabel("ángulo girado según el giroscopio (°)")
        fig.suptitle("Eficacia de los giros en exterior"); fig.tight_layout(); fig.savefig(FIG / "f3_giros.png"); plt.close(fig)

    # ---------------- F4: detecciones
    det = [(r["t"], r["datos"]["class"], r["datos"]["confidence"]) for r in R if r["tipo"] == "detection" and VENTANA[0] <= r["t"] <= VENTANA[1]]
    cl = Counter(c for _, c, _ in det)
    M["detecciones"] = {"total": len(det), "por_clase": dict(cl), "confianza_media": {c: round(float(np.mean([x for _, k, x in det if k == c])), 2) for c in cl}}
    if det:
        fig, axs = plt.subplots(1, 2, figsize=(9.5, 3.4))
        for c, colr in zip(sorted(cl), (VERDE, NAR, AZUL, ROJO, "#805ad5")):
            xs = [(t - t_ini) / 60 for t, k, _ in det if k == c]; ys = [x for _, k, x in det if k == c]
            axs[0].scatter(xs, ys, s=8, color=colr, label=f"{c} (n={len(xs)})")
        axs[0].set_xlabel("minutos"); axs[0].set_ylabel("confianza"); axs[0].legend(fontsize=7)
        axs[1].boxplot([[x for _, k, x in det if k == c] for c in sorted(cl)], tick_labels=[c.replace("col_", "") for c in sorted(cl)])
        axs[1].set_ylabel("confianza")
        fig.suptitle("Detecciones del modelo activo (agrobot_plagasx_det.pt) y del detector de macetas"); fig.tight_layout(); fig.savefig(FIG / "f4_detecciones.png"); plt.close(fig)

    # ---------------- F5: plantas escaneadas (fotos de la visión activa) y capturas de plaga
    fv = sorted(glob.glob(str(HOME / "vision_activa" / "fotos" / "20261027_21*_maceta_frente.jpg")) + glob.glob(str(HOME / "vision_activa" / "fotos" / "20261027_22*_maceta_frente.jpg")))
    fv = [f for f in fv if "215" <= os.path.basename(f)[9:12] or True][:8]
    insp = [r for r in R if r["tipo"] == "vision_activa" and r["datos"].get("tipo") == "inspeccion"]
    fotos_p = [r["datos"].get("foto") for r in insp if r["datos"].get("foto") and Path(r["datos"]["foto"]).exists()]
    if fotos_p:
        k = len(fotos_p); cols = min(3, k); rows = math.ceil(k / cols)
        fig, axs = plt.subplots(rows, cols, figsize=(3.6 * cols, 2.9 * rows)); axs = np.atleast_1d(axs).ravel()
        for ax, f, p in zip(axs, fotos_p, plantas + [{}] * 10):
            ax.imshow(cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB)); ax.axis("off"); ax.grid(False)
            ax.set_title(f"P{p.get('n', '?')} · especie CLIP: {p.get('especie')} (p_col {p.get('p_col')})", fontsize=8)
        for ax in axs[len(fotos_p):]:
            ax.axis("off")
        fig.suptitle("Plantas escaneadas por la visión activa (vista frontal a ~45–55 cm)"); fig.tight_layout(); fig.savefig(FIG / "f5_plantas.png"); plt.close(fig)
    caps = sorted(glob.glob(str(RAIZ / "jetson" / "backend" / "captures" / "17931*_col_*.jpg")))
    caps = [c for c in caps if VENTANA[0] - 1 <= float(mono(int(os.path.basename(c).split("_")[0]) / 1000)) <= VENTANA[1] + 1]
    M["capturas_plaga_archivos"] = len(caps)
    if caps:
        sel_c = caps[:: max(1, len(caps) // 6)][:6]
        fig, axs = plt.subplots(2, 3, figsize=(10, 5)); axs = axs.ravel()
        for ax, f in zip(axs, sel_c):
            ax.imshow(cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB)); ax.axis("off"); ax.grid(False); ax.set_title(os.path.basename(f).split("_", 1)[1][:-4], fontsize=8)
        for ax in axs[len(sel_c):]:
            ax.axis("off")
        fig.suptitle("Capturas guardadas por avisos de plaga (con la caja dibujada por la cámara)"); fig.tight_layout(); fig.savefig(FIG / "f5b_capturas_plaga.png"); plt.close(fig)

    # ---------------- F6: fotogramas de los videos
    cuadros = []
    for c in CAMPOS:
        for v in glob.glob(str(c / "*.avi")):
            cap = cv2.VideoCapture(v); n = int(cap.get(7)); fps = cap.get(5) or 10.0
            ini_v = float(re.search(r"video_(\d+)", v).group(1)) / 1e9 if re.search(r"video_(\d+)", v) else None
            marcas = np.linspace(int(2 * fps), n - int(2 * fps), 8 if n > 2000 else 4).astype(int)
            for fr in marcas:
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(fr)); ok, im = cap.read()
                if ok:
                    cuadros.append((os.path.basename(v)[:14], fr / fps, im))
            M.setdefault("videos", []).append({"archivo": os.path.basename(v), "cuadros": n, "duracion_s": round(n / fps), "resolucion": f"{int(cap.get(3))}x{int(cap.get(4))}", "fps": round(fps, 1), "MB": round(os.path.getsize(v) / 1e6)})
    if cuadros:
        cols = 4; rows = math.ceil(len(cuadros) / cols)
        fig, axs = plt.subplots(rows, cols, figsize=(3.2 * cols, 2.5 * rows)); axs = np.atleast_1d(axs).ravel()
        for ax, (nom, t, im) in zip(axs, cuadros):
            ax.imshow(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)); ax.axis("off"); ax.grid(False); ax.set_title(f"{nom[-8:]} · t={t:.0f} s", fontsize=7)
        for ax in axs[len(cuadros):]:
            ax.axis("off")
        fig.suptitle("Fotogramas del video de la prueba (640×480, 10 fps)"); fig.tight_layout(); fig.savefig(FIG / "f6_video.png"); plt.close(fig)

    # ---------------- F7: clasificador de entorno
    E = [e for e in jsonl(HOME / "entorno" / "20261027.jsonl") if e.get("rasgos") and e.get("puntuacion") is not None and VENTANA[0] <= float(mono(e["t"])) <= VENTANA[1]]
    if E:
        te = np.array([(float(mono(e["t"])) - t_ini) / 60 for e in E])
        fig, axs = plt.subplots(2, 1, figsize=(9, 4.8), sharex=True)
        for k, c in zip(("brillo", "cielo", "verde", "rugosidad"), (NAR, AZUL, VERDE, GRIS)):
            axs[0].plot(te, [e["rasgos"][k] for e in E], color=c, lw=1, label=k)
        axs[0].legend(fontsize=7, ncol=4); axs[0].set_ylabel("rasgos de la cámara")
        axs[1].plot(te, [e["puntuacion"] for e in E], color=NAR, lw=1, label="puntuación automática (+1 interior, −1 exterior)")
        axs[1].axhline(0, color="k", lw=0.6); axs[1].set_xlabel("minutos"); axs[1].legend(fontsize=7); axs[1].set_ylabel("puntuación")
        fig.suptitle("Clasificador automático de entorno durante la prueba de exterior"); fig.tight_layout(); fig.savefig(FIG / "f7_entorno.png"); plt.close(fig)
        M["entorno_auto"] = {"muestras": len(E), "sugerido": dict(Counter(e.get("sugerido") for e in E)), "modo_en_uso": dict(Counter(e.get("modo") for e in E)),
                             "etiqueta_operador": dict(Counter(e.get("etiqueta") for e in E)), "puntuacion_media": round(float(np.mean([e["puntuacion"] for e in E])), 2),
                             "rasgos_medios": {k: round(float(np.mean([e["rasgos"].get(k) or 0 for e in E])), 3) for k in ("brillo", "cielo", "verde", "rugosidad", "cercano")}}

    # ---------------- F8: scans del LIDAR crudos, exterior vs interior
    def stats_scans(carpeta):
        sin_ret, med, tot_s = [], [], 0
        for f in sorted(glob.glob(str(carpeta / "scans_*.npz"))):
            z = np.load(f)
            D = z["distancias_m"].astype(float)
            ok = np.isfinite(D) & (D > 0.05)
            sin_ret += list(1 - ok.mean(axis=1)); med += [float(np.median(d[o])) if o.any() else np.nan for d, o in zip(D, ok)]; tot_s += D.shape[0]
        return np.array(sin_ret), np.array(med), tot_s
    s_ext, m_ext, n_ext = stats_scans(DATOS); s_int, m_int, n_int = stats_scans(DATOS_INTERIOR)
    M["scans"] = {"exterior": {"n": n_ext, "sin_retorno_pct": round(100 * float(np.nanmean(s_ext)), 1), "mediana_alcance_m": round(float(np.nanmedian(m_ext)), 2)},
                  "interior_mismo_dia": {"n": n_int, "sin_retorno_pct": round(100 * float(np.nanmean(s_int)), 1), "mediana_alcance_m": round(float(np.nanmedian(m_int)), 2)}}
    fig = plt.figure(figsize=(9.5, 3.8))
    ax = fig.add_subplot(1, 3, 1, projection="polar")
    z = np.load(sorted(glob.glob(str(DATOS / "scans_*.npz")))[len(glob.glob(str(DATOS / "scans_*.npz"))) // 2]); D = z["distancias_m"].astype(float)
    k = int(np.nanargmin([np.nanmedian(d) for d in D])); ax.scatter(np.radians(z["angulos_deg"]), D[k], s=3, color=VERDE); ax.set_ylim(0, 6); ax.set_title("Scan en exterior", fontsize=9)
    z = np.load(sorted(glob.glob(str(DATOS_INTERIOR / "scans_*.npz")))[len(glob.glob(str(DATOS_INTERIOR / "scans_*.npz"))) // 2]); D = z["distancias_m"].astype(float)
    ax2 = fig.add_subplot(1, 3, 2, projection="polar"); ax2.scatter(np.radians(z["angulos_deg"]), D[0], s=3, color=AZUL); ax2.set_ylim(0, 6); ax2.set_title("Scan en interior", fontsize=9)
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.hist([100 * s_ext, 100 * s_int], bins=np.arange(0, 101, 5), label=[f"exterior (n={n_ext})", f"interior (n={n_int})"], color=[VERDE, AZUL], alpha=0.8)
    ax3.set_xlabel("% de rayos sin retorno por scan"); ax3.set_ylabel("scans"); ax3.legend(fontsize=7)
    fig.suptitle("LIDAR: scans crudos y rayos sin retorno"); fig.tight_layout(); fig.savefig(FIG / "f8_scans.png"); plt.close(fig)

    (OUT / "metricas.json").write_text(json.dumps(M, indent=1, ensure_ascii=False, default=str))
    print(json.dumps(M, indent=1, ensure_ascii=False, default=str)[:6000])


if __name__ == "__main__":
    main()
