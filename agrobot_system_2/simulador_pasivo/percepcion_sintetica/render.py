"""Renderizador sintético de INTERIORES con verdad por píxel (trazado de rayos, numpy).

Por qué uno propio: `mundo.Mundo.camara()` pinta suelo exterior y las macetas como DISCOS
PLANOS sin altura. Una maceta de 25 cm de alto se ve en la cámara como una pared vertical, y
eso es lo que el modelo tiene que aprender a reconocer. Aquí los objetos tienen volumen.

GEOMETRÍA DE CÁMARA: idéntica a `sendero.Cenital` (la que usa el robot para proyectar la
imagen al suelo): cámara a `altura` m, inclinada `inclinacion` grados hacia abajo, campo de
visión horizontal `hfov`, `adelante` m por delante del centro. El test
`test_percepcion_sintetica.py::test_geometria_igual_a_sendero_cenital` lo comprueba. Si esto
divergiera, un píxel segmentado como "suelo libre" se proyectaría a otro sitio en el robot.

CLASES (índices de la máscara):
  0 fondo            pared, techo, cielo: lo que no es suelo ni objeto
  1 camino_libre     suelo transitable
  2 planta_maceta    maceta + planta: NO pisar, NO rozar
  3 borde_bajo       bordillo / escalón / montículo de 4-8 cm: el LIDAR (a 18 cm) NO lo ve
  4 obstaculo        caja, silla, mochila: obstáculo alto
Son las 5 clases del dataset de Codex (0 fondo, 1 sendero, 2 zona_cultivo_no_pisar,
3 bancal_borde_elevado, 4 obstaculo_visible), para que el modelo sea intercambiable.

LÍMITES: es un render sintético sencillo (sombreado difuso + ruido), no fotorrealista. Mide
si el modelo aprende la GEOMETRÍA y el contraste, no si funciona con fotos reales. Nunca se
ha evaluado aquí sobre fotos reales del huerto (están en el PC del operador, no en la Jetson).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

CLASES = ["fondo", "camino_libre", "planta_maceta", "borde_bajo", "obstaculo"]
COLORES_CLASE = np.array([[70, 70, 70], [60, 140, 230], [60, 180, 60], [230, 170, 40], [200, 50, 50]], np.uint8)  # RGB

CAMARA = dict(altura=0.40, inclinacion=15.5, hfov=71.0, adelante=0.42)   # agrobot_config.json
ALTURA_PARED = 2.5


# ----------------------------------------------------------------------------- escena
@dataclass
class Cilindro:
    x: float
    y: float
    r: float
    h: float
    clase: int                  # 2 maceta, 4 obstaculo (pata de silla...)
    color: tuple                # RGB 0..255
    copa_r: float = 0.0         # radio de la planta (0 = sin planta)
    copa_h: float = 0.0
    plagas: list = field(default_factory=list)   # [(ang, altura_rel, tipo)] sobre la copa
    id: int = -1


@dataclass
class Caja:
    x: float
    y: float
    lx: float
    ly: float
    h: float
    ang: float
    clase: int                  # 3 borde bajo, 4 obstaculo
    color: tuple
    id: int = -1


@dataclass
class Escena:
    paredes: list               # [(x0, y0, x1, y1)]
    cilindros: list
    cajas: list
    suelo: dict                 # tipo, color, color2, escala, junta
    luz: dict                   # ambiente, dir (azimut,elevacion), color
    nombre: str = ""
    limites: tuple = (-1.0, -3.0, 6.0, 3.0)

    def objetos(self):
        return list(self.cilindros) + list(self.cajas)


# ----------------------------------------------------------------------------- utilidades
def _hash(x, y):
    h = np.sin(x * 127.1 + y * 311.7) * 43758.5453
    return h - np.floor(h)


def _ruido(x, y, escala):
    """Ruido de valor suave 0..1."""
    x, y = x / escala, y / escala
    x0, y0 = np.floor(x), np.floor(y)
    fx, fy = x - x0, y - y0
    fx, fy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    a, b = _hash(x0, y0), _hash(x0 + 1, y0)
    c, d = _hash(x0, y0 + 1), _hash(x0 + 1, y0 + 1)
    return a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + c * (1 - fx) * fy + d * fx * fy


def rayos(ancho, alto, cam, x, y, psi, cabeceo_extra=0.0, balanceo=0.0):
    """Direcciones de rayo (alto, ancho, 3) y origen. Marco mundo: x adelante, y izquierda, z arriba."""
    f = (ancho / 2) / math.tan(math.radians(cam["hfov"]) / 2)
    ys, xs = np.mgrid[0:alto, 0:ancho].astype(np.float32)
    xn = (xs + 0.5 - ancho / 2) / f
    yn = (ys + 0.5 - alto / 2) / f
    phi = math.radians(cam["inclinacion"] + cabeceo_extra)
    c, s = math.cos(psi), math.sin(psi)
    fwd = np.array([c * math.cos(phi), s * math.cos(phi), -math.sin(phi)], np.float32)
    der = np.array([s, -c, 0.0], np.float32)
    # `aba` apunta hacia ABAJO en la imagen: con z positivo = arriba en el mundo, su componente z es NEGATIVA.
    aba = np.array([-c * math.sin(phi), -s * math.sin(phi), -math.cos(phi)], np.float32)
    if balanceo:                                   # giro de la cámara sobre su eje óptico
        cr, sr = math.cos(balanceo), math.sin(balanceo)
        der, aba = cr * der + sr * aba, -sr * der + cr * aba
    d = fwd + xn[..., None] * der + yn[..., None] * aba
    origen = np.array([x + cam["adelante"] * c, y + cam["adelante"] * s, cam["altura"]], np.float32)
    return d.astype(np.float32), origen, f


# ----------------------------------------------------------------------------- render
def render(esc: Escena, x, y, psi, rng, ancho=320, alto=240, cam=None, cabeceo_extra=0.0,
           balanceo=0.0, efectos=True, pegar_plagas=True):
    """Devuelve dict: rgb (uint8 alto,ancho,3), clase (uint8), instancia (int16, -1 = ninguna),
    plagas [(x0,y0,x1,y1,tipo)] en píxeles, profundidad (float32, m a lo largo del rayo)."""
    cam = dict(CAMARA if cam is None else cam)
    d, o, f = rayos(ancho, alto, cam, x, y, psi, cabeceo_extra, balanceo)
    mejor = np.full((alto, ancho), np.inf, np.float32)
    clase = np.zeros((alto, ancho), np.uint8)
    inst = np.full((alto, ancho), -1, np.int16)
    col = np.zeros((alto, ancho, 3), np.float32)
    px = np.zeros((alto, ancho), np.float32)          # punto de impacto (para texturas)
    py = np.zeros((alto, ancho), np.float32)
    pz = np.zeros((alto, ancho), np.float32)
    nrm = np.zeros((alto, ancho, 3), np.float32)
    es_suelo = np.zeros((alto, ancho), bool)
    es_copa = np.zeros((alto, ancho), bool)
    dz = d[..., 2]
    dxy = np.hypot(d[..., 0], d[..., 1])
    az, el = math.radians(esc.luz["dir"][0]), math.radians(esc.luz["dir"][1])
    L = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)], np.float32)

    def actualizar(t, mascara, etiqueta, ident, normal):
        nonlocal mejor
        ok = mascara & (t > 0.02) & (t < mejor)
        if not ok.any():
            return
        mejor = np.where(ok, t, mejor)
        clase[ok] = etiqueta
        inst[ok] = ident
        es_suelo[ok] = False
        es_copa[ok] = False
        px[ok] = o[0] + t[ok] * d[..., 0][ok]
        py[ok] = o[1] + t[ok] * d[..., 1][ok]
        pz[ok] = o[2] + t[ok] * d[..., 2][ok]
        if normal is not None:
            nrm[ok] = normal if np.ndim(normal) == 1 else normal[ok]

    # --- suelo z=0
    with np.errstate(divide="ignore", invalid="ignore"):
        t_suelo = np.where(dz < -1e-6, -o[2] / dz, np.inf).astype(np.float32)
    actualizar(t_suelo, np.isfinite(t_suelo), 1, -1, np.array([0, 0, 1], np.float32))
    es_suelo |= (clase == 1)

    # --- paredes (planos verticales, 0..ALTURA_PARED)
    for (x0, y0, x1, y1) in esc.paredes:
        ex, ey = x1 - x0, y1 - y0
        den = d[..., 0] * ey - d[..., 1] * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x0 - o[0]) * ey - (y0 - o[1]) * ex) / den
            u = ((x0 - o[0]) * d[..., 1] - (y0 - o[1]) * d[..., 0]) / den
        z = o[2] + t * dz
        ok = (np.abs(den) > 1e-9) & (u >= 0) & (u <= 1) & (z >= 0) & (z <= ALTURA_PARED)
        L2 = math.hypot(ex, ey) + 1e-9
        n = np.array([ey / L2, -ex / L2, 0.0], np.float32)
        actualizar(np.where(ok, t, np.inf).astype(np.float32), ok, 0, -1, n)

    # --- cilindros (maceta con planta, patas)
    for cil in esc.cilindros:
        fx, fy = o[0] - cil.x, o[1] - cil.y
        a = d[..., 0] ** 2 + d[..., 1] ** 2
        b = 2 * (fx * d[..., 0] + fy * d[..., 1])
        # cuerpo
        for (rad, z0, z1, etiqueta, copa) in ((cil.r, 0.0, cil.h, cil.clase, False),
                                               (cil.copa_r, cil.h, cil.h + cil.copa_h, 2, True)):
            if rad <= 0 or z1 <= z0:
                continue
            c_ = fx * fx + fy * fy - rad * rad
            disc = b * b - 4 * a * c_
            ok0 = disc >= 0
            sq = np.sqrt(np.where(ok0, disc, 0.0))
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (-b - sq) / (2 * a)
            z = o[2] + t * dz
            ok = ok0 & (t > 0) & (z >= z0) & (z <= z1)
            hx, hy = o[0] + t * d[..., 0], o[1] + t * d[..., 1]
            n = np.stack([(hx - cil.x) / rad, (hy - cil.y) / rad, np.zeros_like(hx)], -1).astype(np.float32)
            actualizar(np.where(ok, t, np.inf).astype(np.float32), ok, etiqueta, cil.id, n)
            if copa:
                es_copa |= (inst == cil.id) & (clase == 2) & (pz > cil.h - 1e-3)
            # tapa superior
            with np.errstate(divide="ignore", invalid="ignore"):
                tt = np.where(dz != 0, (z1 - o[2]) / dz, np.inf)
            hx, hy = o[0] + tt * d[..., 0], o[1] + tt * d[..., 1]
            oktop = (tt > 0) & ((hx - cil.x) ** 2 + (hy - cil.y) ** 2 <= rad * rad)
            actualizar(np.where(oktop, tt, np.inf).astype(np.float32), oktop, etiqueta, cil.id,
                       np.array([0, 0, 1], np.float32))
            if copa:
                es_copa |= (inst == cil.id) & (pz >= z1 - 1e-3)

    # --- cajas orientadas (método de las losas en el marco de la caja)
    for cj in esc.cajas:
        c, s = math.cos(-cj.ang), math.sin(-cj.ang)
        ox, oy = (o[0] - cj.x) * c - (o[1] - cj.y) * s, (o[0] - cj.x) * s + (o[1] - cj.y) * c
        dx, dy = d[..., 0] * c - d[..., 1] * s, d[..., 0] * s + d[..., 1] * c
        t0 = np.full((alto, ancho), -np.inf, np.float32)
        t1 = np.full((alto, ancho), np.inf, np.float32)
        eje_n = np.zeros((alto, ancho, 3), np.float32)
        for (oo, dd, lo, hi, k) in ((ox, dx, -cj.lx / 2, cj.lx / 2, 0), (oy, dy, -cj.ly / 2, cj.ly / 2, 1),
                                    (o[2], dz, 0.0, cj.h, 2)):
            with np.errstate(divide="ignore", invalid="ignore"):
                ta, tb = (lo - oo) / dd, (hi - oo) / dd
            tmin, tmax = np.minimum(ta, tb), np.maximum(ta, tb)
            paralelo = np.abs(dd) < 1e-9
            dentro = (oo >= lo) & (oo <= hi)
            tmin = np.where(paralelo, np.where(dentro, -np.inf, np.inf), tmin)
            tmax = np.where(paralelo, np.where(dentro, np.inf, -np.inf), tmax)
            mas = tmin > t0
            t0 = np.where(mas, tmin, t0)
            eje_n[mas] = 0
            sg = -np.sign(dd)
            eje_n[..., k] = np.where(mas, sg, eje_n[..., k])
            t1 = np.minimum(t1, tmax)
        ok = (t0 <= t1) & (t0 > 0)
        # normal al marco mundo
        nx = eje_n[..., 0] * math.cos(cj.ang) - eje_n[..., 1] * math.sin(cj.ang)
        ny = eje_n[..., 0] * math.sin(cj.ang) + eje_n[..., 1] * math.cos(cj.ang)
        n = np.stack([nx, ny, eje_n[..., 2]], -1).astype(np.float32)
        actualizar(np.where(ok, t0, np.inf).astype(np.float32), ok, cj.clase, cj.id, n)

    # ------------------------------------------------------------------ color
    lum = esc.luz["ambiente"] + (1 - esc.luz["ambiente"]) * np.clip((nrm * L).sum(-1), 0, 1)
    # --- suelo
    s = esc.suelo
    sx, sy = px, py
    base = np.array(s["color"], np.float32)
    if s["tipo"] == "baldosa":
        g = s["escala"]
        junta = ((np.abs(((sx / g) % 1.0) - 0.5) > 0.5 - s["junta"]) | (np.abs(((sy / g) % 1.0) - 0.5) > 0.5 - s["junta"]))
        par = ((np.floor(sx / g) + np.floor(sy / g)) % 2 == 0)
        c1, c2 = base, np.array(s["color2"], np.float32)
        suelo_col = np.where(par[..., None], c1, c2) * (0.93 + 0.14 * _ruido(sx, sy, g * 0.6)[..., None])
        suelo_col = np.where(junta[..., None], suelo_col * 0.62, suelo_col)
    elif s["tipo"] == "madera":
        g = s["escala"]
        tabla = np.floor(sy / g)
        tono = 0.85 + 0.3 * _hash(tabla, np.floor(sx / (g * 9)))
        veta = 0.94 + 0.12 * _ruido(sx * 9, sy * 0.7, g)
        suelo_col = base * (tono * veta)[..., None]
        suelo_col = np.where((np.abs(((sy / g) % 1.0) - 0.5) > 0.49)[..., None], suelo_col * 0.6, suelo_col)
    elif s["tipo"] == "alfombra":
        suelo_col = base * (0.85 + 0.3 * _ruido(sx, sy, 0.03)[..., None])
    else:                                          # hormigón
        suelo_col = base * (0.88 + 0.24 * _ruido(sx, sy, 0.25)[..., None]) * (0.95 + 0.1 * _ruido(sx, sy, 0.02)[..., None])
    # sombra suave bajo los objetos
    sombra = np.ones((alto, ancho), np.float32)
    for ob in esc.cilindros:
        r2 = (sx - ob.x) ** 2 + (sy - ob.y) ** 2
        sombra *= 1 - 0.45 * np.exp(-r2 / (2 * (ob.r * 1.6) ** 2)) * (ob.h > 0.05)
    for ob in esc.cajas:
        r2 = (sx - ob.x) ** 2 + (sy - ob.y) ** 2
        sombra *= 1 - 0.40 * np.exp(-r2 / (2 * (max(ob.lx, ob.ly) * 0.8) ** 2)) * (ob.h > 0.05)
    # --- materiales por clase
    col[:] = np.array([170, 170, 175], np.float32)                    # pared por defecto
    parc = (clase == 0) & np.isfinite(mejor)
    col[parc] = (np.array(esc.luz.get("pared", (200, 195, 185)), np.float32)
                 * (0.75 + 0.25 * np.clip(pz[parc] / 2.0, 0, 1))[..., None])
    sue = es_suelo & (clase == 1)
    col[sue] = suelo_col[sue] * sombra[sue][..., None]
    for ob in esc.cilindros:
        m = inst == ob.id
        if not m.any():
            continue
        base_ob = np.array(ob.color, np.float32)
        verde = np.array([45, 120 + 40 * _hash(ob.x, ob.y), 55], np.float32)
        copa = m & es_copa
        cuerpo = m & ~copa
        col[cuerpo] = base_ob * (0.9 + 0.2 * _ruido(px[cuerpo], pz[cuerpo] * 3, 0.04))[..., None]
        if copa.any():
            hoja = 0.65 + 0.7 * _ruido(px[copa] * 2.0, py[copa] * 2.0 + pz[copa] * 5, 0.025)
            col[copa] = verde * hoja[..., None]
    for ob in esc.cajas:
        m = inst == ob.id
        if m.any():
            col[m] = np.array(ob.color, np.float32) * (0.92 + 0.16 * _ruido(px[m], py[m] + pz[m], 0.05))[..., None]
    # --- iluminación y niebla
    col *= lum[..., None]
    dist = mejor * np.sqrt((d ** 2).sum(-1))
    nb = np.clip(1 - np.exp(-0.05 * np.maximum(dist - 1.5, 0)), 0, 0.35)[..., None]
    col = col * (1 - nb) + np.array([215, 212, 205], np.float32) * nb
    # techo / cielo: por encima de lo que golpea (rayo que sube y no toca nada)
    sin_impacto = ~np.isfinite(mejor)
    col[sin_impacto] = np.array(esc.luz.get("techo", (225, 225, 228)), np.float32)

    # --- plagas: sprites sobre la copa (se pegan en la imagen, escalados por distancia)
    cajas_plaga = []
    if pegar_plagas:
        cajas_plaga = _pegar_plagas(col, esc, o, d, f, ancho, alto, rng, cam, x, y, psi, clase, inst, mejor)

    img = np.clip(col, 0, 255)
    prof = np.where(np.isfinite(mejor), dist, np.inf).astype(np.float32)
    if efectos:
        img = _camara_real(img, rng)
    return {"rgb": img.astype(np.uint8), "clase": clase, "instancia": inst, "plagas": cajas_plaga,
            "profundidad": prof}


TIPOS_PLAGA = ["pulgon", "oruga", "escarabajo"]


def _proyectar(p, o, psi, cam, ancho, alto, cabeceo_extra=0.0):
    """Punto del mundo -> (u, v, profundidad). Mismo modelo que `rayos`."""
    f = (ancho / 2) / math.tan(math.radians(cam["hfov"]) / 2)
    phi = math.radians(cam["inclinacion"] + cabeceo_extra)
    c, s = math.cos(psi), math.sin(psi)
    fwd = np.array([c * math.cos(phi), s * math.cos(phi), -math.sin(phi)])
    der = np.array([s, -c, 0.0])
    aba = np.array([-c * math.sin(phi), -s * math.sin(phi), -math.cos(phi)])
    v = np.asarray(p) - o
    z = v @ fwd
    if z <= 0.05:
        return None
    return ancho / 2 + f * (v @ der) / z, alto / 2 + f * (v @ aba) / z, z


def _pegar_plagas(col, esc, o, d, f, ancho, alto, rng, cam, x, y, psi, clase, inst, mejor):
    import cv2
    cajas = []
    for cil in esc.cilindros:
        for (ang, hrel, tipo) in cil.plagas:
            rad = cil.copa_r * 0.92
            p = (cil.x + rad * math.cos(ang), cil.y + rad * math.sin(ang), cil.h + cil.copa_h * hrel)
            pr = _proyectar(p, o, psi, cam, ancho, alto)
            if pr is None:
                continue
            u, v, z = pr
            iu, iv = int(round(u)), int(round(v))
            if not (2 <= iu < ancho - 2 and 2 <= iv < alto - 2):
                continue
            # visible solo si el pixel pertenece a la copa de ESA planta (no está tapada)
            if inst[iv, iu] != cil.id or clase[iv, iu] != 2:
                continue
            tam = max(2.0, 0.022 * f / z)              # 2,2 cm de insecto a esa distancia
            if tam < 2.2:
                continue
            _dibujar_plaga(col, iu, iv, tam, tipo, rng, cv2)
            r = tam * (1.8 if tipo == "oruga" else 1.1)
            cajas.append((iu - r, iv - tam * 0.9, iu + r, iv + tam * 0.9, TIPOS_PLAGA.index(tipo)))
    if cajas or rng.random() < 0.5:
        _pegar_senuelos(col, esc, f, rng, clase, inst, mejor, d, o)
    return cajas


def _pegar_senuelos(col, esc, f, rng, clase, inst, mejor, d, o):
    """Manchas de hoja y motas de tierra que NO son plaga: obligan al detector a mirar la forma,
    no solo 'algo pequeño y oscuro sobre una hoja'. No llevan etiqueta."""
    import cv2
    ys, xs = np.nonzero((clase == 2) & (inst >= 0))
    if xs.size < 50:
        return
    for _ in range(int(rng.integers(2, 10))):
        k = int(rng.integers(xs.size))
        z = float(mejor[ys[k], xs[k]] * np.sqrt((d[ys[k], xs[k]] ** 2).sum()))
        rad = max(1, int(round(0.012 * f / max(z, 0.3))))
        color = [(40, 60, 90), (30, 40, 30), (150, 140, 100), (60, 90, 50)][int(rng.integers(4))]
        cv2.circle(col, (int(xs[k]), int(ys[k])), rad, color, -1)


def _dibujar_plaga(col, u, v, tam, tipo, rng, cv2):
    t = int(round(tam))
    if tipo == "pulgon":              # grupo de puntos verde claro/negro
        for _ in range(5):
            du, dv = rng.integers(-t, t + 1), rng.integers(-t // 2, t // 2 + 1)
            cv2.circle(col, (u + du, v + dv), max(1, t // 3), (210, 230, 120) if rng.random() < .6 else (35, 35, 35), -1)
    elif tipo == "oruga":             # cuerpo alargado a segmentos, amarillo-verde con cabeza oscura
        for k in range(-3, 4):
            cv2.circle(col, (u + int(k * t * 0.5), v), max(1, t // 2), (110, 220, 190), -1)
        cv2.circle(col, (u + int(3.5 * t * 0.5), v), max(1, t // 2), (40, 50, 60), -1)
    else:                             # escarabajo: óvalo oscuro con brillo
        cv2.ellipse(col, (u, v), (max(2, t), max(1, int(t * 0.7))), 0, 0, 360, (25, 25, 40), -1)
        cv2.circle(col, (u - t // 3, v - t // 3), max(1, t // 4), (180, 180, 190), -1)


def _camara_real(img, rng):
    """Degradado de cámara: exposición, balance, desenfoque, ruido y JPEG."""
    import cv2
    g = rng.uniform(0.7, 1.25)
    bal = rng.uniform(0.9, 1.1, 3)
    img = img * g * bal
    if rng.random() < 0.5:
        k = rng.choice([3, 5])
        img = cv2.GaussianBlur(img, (k, k), rng.uniform(0.5, 1.2))
    img = img + rng.normal(0, rng.uniform(1.5, 6.0), img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    q = int(rng.integers(45, 95))
    ok, buf = cv2.imencode(".jpg", img[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, q])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)[..., ::-1].astype(np.float32)
