"""Compara detectores sobre fotos del robot etiquetadas a mano, sin depender de ultralytics.
Las clases se comparan por NOMBRE (v8 tiene 11 clases y v9 tiene 12). col_sana no cuenta como plaga."""
import math

NO_PLAGA = {"col_sana"}


def iou(a, b):
    """Cajas (x, y, w, h) con centro normalizado."""
    ix = min(a[0] + a[2] / 2, b[0] + b[2] / 2) - max(a[0] - a[2] / 2, b[0] - b[2] / 2)
    iy = min(a[1] + a[3] / 2, b[1] + b[3] / 2) - max(a[1] - a[3] / 2, b[1] - b[3] / 2)
    if ix <= 0 or iy <= 0:
        return 0.0
    inter = ix * iy
    return inter / (a[2] * a[3] + b[2] * b[3] - inter)


def wilson(k, n, z=1.96):
    """Intervalo de confianza del 95 % para k aciertos en n casos; None si no hay casos."""
    if n == 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    m = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return round((c - m) / d, 3), round((c + m) / d, 3)


def _razon(k, n):
    return {"valor": round(k / n, 3) if n else None, "k": k, "n": n, "ic95": wilson(k, n)}


def evaluar(fotos, iou_min=0.3):
    """fotos: lista de {"real": [(clase, x, y, w, h)], "pred": [(clase, conf, x, y, w, h)]}.
    Cada caja predicha empareja como máximo una real. Devuelve exhaustividad y precisión por caja, y resultados por imagen."""
    tp_any = tp_clase = n_real = n_pred = 0
    img_con, img_con_ok, img_sin, img_sin_aviso = 0, 0, 0, 0
    por_clase = {}
    for f in fotos:
        real = [r for r in f["real"] if r[0] not in NO_PLAGA]
        pred = sorted((p for p in f["pred"] if p[0] not in NO_PLAGA), key=lambda p: -p[1])
        n_real += len(real); n_pred += len(pred)
        libres = list(range(len(real)))
        for p in pred:
            candidatos = sorted(((iou(p[2:], real[i][1:]), i) for i in libres), reverse=True)
            if candidatos and candidatos[0][0] >= iou_min:
                # Prefiere una caja real de la misma clase entre las que solapan lo suficiente.
                i = next((i for v, i in candidatos if v >= iou_min and real[i][0] == p[0]), candidatos[0][1])
                libres.remove(i); tp_any += 1
                tp_clase += real[i][0] == p[0]
                por_clase.setdefault(real[i][0], [0, 0])[0] += real[i][0] == p[0]
        for r in real:
            por_clase.setdefault(r[0], [0, 0])[1] += 1
        if real:
            img_con += 1; img_con_ok += bool(pred)
        else:
            img_sin += 1; img_sin_aviso += bool(pred)
    return {"exhaustividad_caja_cualquier_clase": _razon(tp_any, n_real), "exhaustividad_caja_misma_clase": _razon(tp_clase, n_real),
            "precision_caja": _razon(tp_any, n_pred), "imagenes_con_plaga_avisadas": _razon(img_con_ok, img_con),
            "imagenes_sin_plaga_con_falso_aviso": _razon(img_sin_aviso, img_sin),
            "por_clase": {c: _razon(k, n) for c, (k, n) in sorted(por_clase.items())}}


def leer_etiqueta(texto, nombres):
    return [(nombres[int(p[0])], *map(float, p[1:5])) for p in (l.split() for l in texto.splitlines()) if len(p) == 5]
