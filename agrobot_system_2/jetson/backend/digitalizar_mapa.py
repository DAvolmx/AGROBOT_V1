"""Digitaliza un mapa de ocupación del robot (PGM de slam_toolbox o la rejilla de mapa_scan) en un plano vectorial.

Entrada: `mapa.pgm` + `mapa.yaml` (resolución y origen en metros) o una rejilla numpy (-1 desconocido, 0 libre, 100 ocupado).
Proceso: limpia el ruido del LIDAR (filtro de mediana, cierre de rendijas, manchas sueltas) -> extrae los muros como segmentos rectos
(Hough probabilístico, fusión de los colineales y ajuste a los ejes dominantes del edificio) -> mide el espacio libre.
Salidas (carpeta `salida`): `plano.svg` (a escala, 1 unidad = 1 m), `plano.dxf` (LINE de AutoCAD, capa MUROS), `plano.json`
(segmentos, esquinas, área, bounding box), `plano.png` (vista previa con cotas) y `limpio.png`.

Uso:  python3 digitalizar_mapa.py maps/mision_x/mapa.yaml [salida]
Sin dependencias fuera de numpy y OpenCV. NO necesita ROS."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np


# ------------------------------------------------------------------ lectura
def leer_yaml(ruta: Path) -> dict:
    d = {}
    for linea in ruta.read_text().splitlines():
        if ":" not in linea:
            continue
        k, v = linea.split(":", 1)
        d[k.strip()] = v.strip()
    d["resolution"] = float(d.get("resolution", 0.05))
    o = d.get("origin", "[0,0,0]").strip("[]").split(",")
    d["origin"] = [float(x) for x in o[:2]]
    d["negate"] = int(d.get("negate", 0))
    d["occupied_thresh"] = float(d.get("occupied_thresh", 0.65))
    d["free_thresh"] = float(d.get("free_thresh", 0.196))
    return d


def cargar_pgm(yaml_path: str | Path):
    """-> (rejilla int8 con -1/0/100 y fila 0 = norte, resolución m/celda, origen (x, y) de la esquina inferior izquierda)."""
    yp = Path(yaml_path)
    cfg = leer_yaml(yp)
    img = cv2.imread(str(yp.parent / cfg["image"]), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(yp.parent / cfg["image"])
    if img.ndim == 3:
        img = img[..., 0]
    p = img.astype(np.float32) / 255.0
    if not cfg["negate"]:
        p = 1.0 - p                                    # ocupación = 1 - claro
    g = np.full(img.shape, -1, np.int8)
    g[p < cfg["free_thresh"]] = 0
    g[p > cfg["occupied_thresh"]] = 100
    return g, cfg["resolution"], tuple(cfg["origin"])


# ------------------------------------------------------------------ limpieza
def limpiar(grid: np.ndarray, res: float, min_mancha_m: float = 0.12):
    """-> (ocupado, libre) booleanos. Quita puntos sueltos (< min_mancha_m de lado) y cierra rendijas de 1 celda."""
    occ = (grid == 100).astype(np.uint8)
    occ = cv2.medianBlur(occ * 255, 3) > 0                    # sal y pimienta del LIDAR
    occ = cv2.morphologyEx(occ.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
    min_cel = max(3, int((min_mancha_m / res) ** 2 / 2))
    keep = np.zeros_like(occ, bool)
    for i in range(1, n):
        w, h, a = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT], st[i, cv2.CC_STAT_AREA]
        if max(w, h) * res >= min_mancha_m and a >= min_cel:
            keep |= lab == i
    libre = (grid == 0) & ~keep
    return keep, libre


# ------------------------------------------------------------------ segmentos
def _angulo(s):
    return math.atan2(s[3] - s[1], s[2] - s[0]) % math.pi


def _unir(a, b, tol_ang, tol_dist, hueco):
    """Fusión de dos segmentos casi colineales y solapados (o separados por < hueco m); None si no se pueden unir."""
    ang_a = _angulo(a)
    d_ang = abs(_angulo(b) - ang_a)
    if min(d_ang, math.pi - d_ang) > tol_ang:
        return None
    ux, uy = math.cos(ang_a), math.sin(ang_a)
    nx, ny = -uy, ux
    if max(abs((p[0] - a[0]) * nx + (p[1] - a[1]) * ny) for p in ((b[0], b[1]), (b[2], b[3]))) > tol_dist:
        return None
    ta = sorted([a[0] * ux + a[1] * uy, a[2] * ux + a[3] * uy])
    tb = sorted([b[0] * ux + b[1] * uy, b[2] * ux + b[3] * uy])
    if tb[0] - ta[1] > hueco or ta[0] - tb[1] > hueco:
        return None
    t = [p * 1.0 for p in (ta[0], ta[1], tb[0], tb[1])]
    la, lb = math.hypot(a[2] - a[0], a[3] - a[1]), math.hypot(b[2] - b[0], b[3] - b[1])
    off = ((a[0] * nx + a[1] * ny) * la + (b[0] * nx + b[1] * ny) * lb) / (la + lb)
    t0, t1 = min(t), max(t)
    return [t0 * ux + off * nx, t0 * uy + off * ny, t1 * ux + off * nx, t1 * uy + off * ny]


def _fusionar(segs, tol_ang=math.radians(6), tol_dist=0.07, hueco=0.12):
    """Une segmentos casi colineales hasta que no quede ningún par fusionable. Coordenadas en metros."""
    segs = [list(s) for s in segs]
    while True:
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                m = _unir(segs[i], segs[j], tol_ang, tol_dist, hueco)
                if m is not None:
                    segs[i] = m
                    del segs[j]
                    break
            else:
                continue
            break
        else:
            return segs


def _ejes_dominantes(segs):
    """Ángulo (rad, 0..π/2) del edificio: máximo del histograma de ángulos módulo 90° ponderado por longitud."""
    if not segs:
        return 0.0
    h = np.zeros(90)
    for s in segs:
        L = math.hypot(s[2] - s[0], s[3] - s[1])
        a = int(round(math.degrees(_angulo(s)))) % 90
        h[a] += L
    h = np.convolve(np.r_[h[-3:], h, h[:3]], np.ones(5) / 5, "same")[3:-3]
    return math.radians(int(np.argmax(h)))


def _soporte(occ, s, res):
    """Fracción de celdas del segmento (metros, imagen) con un ocupado a <= 2 celdas."""
    n = max(2, int(math.hypot(s[2] - s[0], s[3] - s[1]) / res))
    d = cv2.dilate(occ.astype(np.uint8), np.ones((5, 5), np.uint8))
    h, w = d.shape
    ok = 0
    for i in range(n + 1):
        x = int(round((s[0] + (s[2] - s[0]) * i / n) / res))
        y = int(round((s[1] + (s[3] - s[1]) * i / n) / res))
        ok += 0 <= x < w and 0 <= y < h and d[y, x] > 0
    return ok / (n + 1)


def _suprimir(segs, tol_dist=0.12, tol_ang=math.radians(12), solape=0.5):
    """Quita los segmentos paralelos que otro más largo ya cubre (Hough devuelve varios por cada muro grueso)."""
    segs = sorted(segs, key=lambda s: -math.hypot(s[2] - s[0], s[3] - s[1]))
    keep = []
    for s in segs:
        Ls = math.hypot(s[2] - s[0], s[3] - s[1])
        cubierto = False
        for k in keep:
            d_ang = abs(_angulo(s) - _angulo(k))
            if min(d_ang, math.pi - d_ang) > tol_ang:
                continue
            ux, uy = k[2] - k[0], k[3] - k[1]
            Lk = math.hypot(ux, uy)
            ux, uy = ux / Lk, uy / Lk
            pts = ((s[0], s[1]), (s[2], s[3]))
            if max(abs(-(p[0] - k[0]) * uy + (p[1] - k[1]) * ux) for p in pts) > tol_dist:
                continue
            t = sorted(((p[0] - k[0]) * ux + (p[1] - k[1]) * uy) for p in pts)
            inter = max(0.0, min(t[1], Lk) - max(t[0], 0.0))
            if inter >= solape * Ls:
                cubierto = True
                break
        if not cubierto:
            keep.append(s)
    return keep


def extraer_segmentos(occ: np.ndarray, res: float, min_largo_m: float = 0.40, ajustar_ejes: bool = True):
    """Segmentos de muro en metros con el origen en la esquina superior izquierda de la rejilla (x derecha, y ABAJO)."""
    m = occ.astype(np.uint8) * 255
    lin = cv2.HoughLinesP(m, 1, np.pi / 180, threshold=max(6, int(0.20 / res)), minLineLength=int(min_largo_m / res),
                          maxLineGap=int(0.15 / res))
    if lin is None:
        return [], 0.0
    lin = np.asarray(lin).reshape(-1, 4)
    segs = [[l[0] * res, l[1] * res, l[2] * res, l[3] * res] for l in lin]
    segs = _fusionar(_suprimir(segs))
    eje = _ejes_dominantes(segs)
    if ajustar_ejes:
        out = []
        for s in segs:
            cx, cy = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
            L = math.hypot(s[2] - s[0], s[3] - s[1])
            a = _angulo(s)
            for objetivo in (eje, eje + math.pi / 2, eje + math.pi):
                d = abs((a - objetivo + math.pi / 2) % math.pi - math.pi / 2)
                if d < math.radians(9):
                    a = objetivo
                    break
            ux, uy = math.cos(a), math.sin(a)
            out.append([cx - ux * L / 2, cy - uy * L / 2, cx + ux * L / 2, cy + uy * L / 2])
        segs = _fusionar(out)
    segs = _suprimir(segs)
    segs = [s for s in segs if math.hypot(s[2] - s[0], s[3] - s[1]) >= min_largo_m and _soporte(occ, s, res) >= 0.45]
    return segs, eje


def esquinas(segs, radio=0.15):
    """Puntos donde dos muros se acercan a menos de `radio` m y no son paralelos (vértices del plano)."""
    pts = []
    for i, a in enumerate(segs):
        for b in segs[i + 1:]:
            d_ang = abs(_angulo(a) - _angulo(b))
            if min(d_ang, math.pi - d_ang) < math.radians(25):
                continue
            for pa in ((a[0], a[1]), (a[2], a[3])):
                for pb in ((b[0], b[1]), (b[2], b[3])):
                    if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < radio:
                        pts.append(((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2))
    out = []
    for p in pts:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > radio for q in out):
            out.append(p)
    return out


# ------------------------------------------------------------------ exportación
def a_mundo(p, res, origen, alto):
    """(x, y) en metros de la imagen (y abajo) -> metros en el marco del mapa (y arriba)."""
    return (origen[0] + p[0], origen[1] + (alto * res - p[1]))


def digitalizar(origen_mapa, salida=None, **kw) -> dict:
    """origen_mapa: ruta al .yaml o tupla (rejilla, resolución, origen). Devuelve el resumen y escribe los archivos si hay `salida`."""
    if isinstance(origen_mapa, (str, Path)):
        grid, res, org = cargar_pgm(origen_mapa)
    else:
        grid, res, org = origen_mapa
    alto = grid.shape[0]
    occ, libre = limpiar(grid, res, kw.get("min_mancha_m", 0.12))
    segs_img, eje = extraer_segmentos(occ, res, kw.get("min_largo_m", 0.40), kw.get("ajustar_ejes", True))
    segs = [[*a_mundo((s[0], s[1]), res, org, alto), *a_mundo((s[2], s[3]), res, org, alto)] for s in segs_img]
    cor = [a_mundo(p, res, org, alto) for p in esquinas(segs_img)]
    ys, xs = np.nonzero(occ | libre)
    if len(xs):
        bbox = [round(org[0] + xs.min() * res, 3), round(org[1] + (alto - 1 - ys.max()) * res, 3),
                round(org[0] + (xs.max() + 1) * res, 3), round(org[1] + (alto - ys.min()) * res, 3)]
    else:
        bbox = [0, 0, 0, 0]
    # espacios: componentes del espacio libre separados tras erosionar 25 cm (puertas estrechas dividen en habitaciones)
    er = cv2.erode(libre.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * max(1, int(0.25 / res)) + 1,) * 2))
    n, lab, st, cen = cv2.connectedComponentsWithStats(er, connectivity=8)
    espacios = []
    for i in range(1, n):
        a_m2 = float(st[i, cv2.CC_STAT_AREA]) * res * res
        if a_m2 >= 0.4:
            c = a_mundo((cen[i][0] * res, cen[i][1] * res), res, org, alto)
            espacios.append({"centro": [round(c[0], 2), round(c[1], 2)], "area_m2": round(a_m2, 2)})
    largo = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
    res_d = {"resolucion_m": res, "origen": list(org), "eje_dominante_deg": round(math.degrees(eje), 1), "bbox": bbox,
             "ancho_m": round(bbox[2] - bbox[0], 2), "alto_m": round(bbox[3] - bbox[1], 2),
             "area_libre_m2": round(float(libre.sum()) * res * res, 2), "largo_muros_m": round(largo, 2),
             "n_muros": len(segs), "n_esquinas": len(cor), "espacios": espacios,
             "muros": [[round(v, 3) for v in s] for s in segs], "esquinas": [[round(p[0], 3), round(p[1], 3)] for p in cor]}
    if salida:
        out = Path(salida)
        out.mkdir(parents=True, exist_ok=True)
        (out / "plano.json").write_text(json.dumps(res_d, ensure_ascii=False, indent=1))
        (out / "plano.svg").write_text(_svg(res_d))
        (out / "plano.dxf").write_text(_dxf(res_d))
        cv2.imwrite(str(out / "limpio.png"), np.where(occ, 0, np.where(libre, 255, 160)).astype(np.uint8))
        cv2.imwrite(str(out / "plano.png"), _preview(grid, occ, libre, segs_img, [(c[0], c[1]) for c in esquinas(segs_img)], res, res_d))
    return res_d


def _svg(d) -> str:
    x0, y0, x1, y1 = d["bbox"]
    m = 0.3
    W, H = x1 - x0 + 2 * m, y1 - y0 + 2 * m
    L = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W*100:.0f}" height="{H*100:.0f}" viewBox="0 0 {W:.3f} {H:.3f}">',
         f'<rect width="{W:.3f}" height="{H:.3f}" fill="#fff"/>', '<g stroke="#111" stroke-width="0.06" stroke-linecap="square">']
    for s in d["muros"]:
        L.append(f'<line x1="{s[0]-x0+m:.3f}" y1="{y1-s[1]+m:.3f}" x2="{s[2]-x0+m:.3f}" y2="{y1-s[3]+m:.3f}"/>')
    L.append("</g>")
    for c in d["esquinas"]:
        L.append(f'<circle cx="{c[0]-x0+m:.3f}" cy="{y1-c[1]+m:.3f}" r="0.05" fill="#d33"/>')
    for e in d["espacios"]:
        L.append(f'<text x="{e["centro"][0]-x0+m:.2f}" y="{y1-e["centro"][1]+m:.2f}" font-size="0.22" fill="#058" text-anchor="middle">'
                 f'{e["area_m2"]} m²</text>')
    L.append("</svg>")
    return "\n".join(L)


def _dxf(d) -> str:
    L = ["0", "SECTION", "2", "ENTITIES"]
    for s in d["muros"]:
        L += ["0", "LINE", "8", "MUROS", "10", f"{s[0]:.4f}", "20", f"{s[1]:.4f}", "30", "0.0",
              "11", f"{s[2]:.4f}", "21", f"{s[3]:.4f}", "31", "0.0"]
    for c in d["esquinas"]:
        L += ["0", "POINT", "8", "ESQUINAS", "10", f"{c[0]:.4f}", "20", f"{c[1]:.4f}", "30", "0.0"]
    L += ["0", "ENDSEC", "0", "EOF"]
    return "\n".join(L) + "\n"


def _preview(grid, occ, libre, segs_img, cor_img, res, d, escala=6):
    h, w = grid.shape
    base = np.full((h, w, 3), 190, np.uint8)
    base[libre] = (255, 255, 255)
    base[occ] = (210, 210, 210)
    im = cv2.resize(base, (w * escala, h * escala), interpolation=cv2.INTER_NEAREST)
    k = escala / res
    for s in segs_img:
        cv2.line(im, (int(s[0] * k), int(s[1] * k)), (int(s[2] * k), int(s[3] * k)), (30, 30, 30), 3)
        L = math.hypot(s[2] - s[0], s[3] - s[1])
        if L > 0.8:
            cv2.putText(im, f"{L:.2f} m", (int((s[0] + s[2]) / 2 * k) + 4, int((s[1] + s[3]) / 2 * k) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 60, 0), 1, cv2.LINE_AA)
    for c in cor_img:
        cv2.circle(im, (int(c[0] * k), int(c[1] * k)), 5, (40, 40, 220), -1)
    cv2.putText(im, f"{d['ancho_m']} x {d['alto_m']} m  libre {d['area_libre_m2']} m2  muros {d['n_muros']} ({d['largo_muros_m']} m)",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return im


def grid_de_mundo(esc, res=0.05, margen=0.4):
    """Rasteriza un Escenario del simulador (paredes en metros) como rejilla de ocupación; sirve para probar el digitalizador con verdad conocida."""
    xs = [v for s in esc.paredes for v in (s[0], s[2])]
    ys = [v for s in esc.paredes for v in (s[1], s[3])]
    x0, y0, x1, y1 = min(xs) - margen, min(ys) - margen, max(xs) + margen, max(ys) + margen
    w, h = int((x1 - x0) / res), int((y1 - y0) / res)
    g = np.zeros((h, w), np.int8)
    for s in esc.paredes:
        a = (int((s[0] - x0) / res), int((y1 - s[1]) / res))
        b = (int((s[2] - x0) / res), int((y1 - s[3]) / res))
        cv2.line(g, a, b, 100, 1)
    return g, res, (x0, y0)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    y = Path(sys.argv[1])
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else y.parent / "digital"
    r = digitalizar(y, dest)
    print(json.dumps({k: v for k, v in r.items() if k not in ("muros", "esquinas")}, ensure_ascii=False, indent=1))
    print("Escrito en", dest)
