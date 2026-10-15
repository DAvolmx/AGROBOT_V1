"""Detección clásica del corredor por vegetación o contraste de suelo.

No usa odometría ni verdad de terreno. Está pensada como baseline interpretable
para probar el pasillo de tierra; la cámara real aún requiere validación diurna
y nocturna antes de usarla para movimiento autónomo.
"""
import numpy as np


def detectar_centros(image):
    """Devuelve ``(centros, fuente)``; cada centro es ``(x_px, y_px)``.

    Primero usa dos bordes verdes. Si no hay evidencia suficiente, busca desde
    el centro hacia afuera el cambio persistente de luminancia/color que forman
    los caballones. Requiere bordes a ambos lados y un ancho de imagen plausible.
    """
    h, w, _ = image.shape
    r, g, b = (image[:, :, i].astype(np.int16) for i in range(3))
    green = (g > 40) & (g > r * 1.14) & (g > b * 1.12) & (r < 205)
    rows = range(int(h * .30), int(h * .98), max(1, h // 40))
    green_centers = {}
    for y in rows:
        left = np.flatnonzero(green[y, :w // 2])
        right = np.flatnonzero(green[y, w // 2:]) + w // 2
        if left.size and right.size:
            gap = int(right[0] - left[-1])
            if max(16, int(w * .12)) <= gap <= int(w * .97):
                green_centers[y] = (float(left[-1] + right[0]) / 2.0, y)

    # Adaptive row reference handles exposure changes. Three adjacent pixels
    # suppress isolated clods and image noise while retaining raised furrows.
    luminance = .299 * r + .587 * g + .114 * b
    soil_centers = {}
    for y in rows:
        row = luminance[y]
        ref = float(np.median(row[int(w * .42):int(w * .58)]))
        if not 25 <= ref <= 240:
            continue
        threshold = max(13.0, ref * .12)
        left = None
        for x in range(w // 2 - 1, 2, -1):
            if np.all(np.abs(row[x - 2:x + 1] - ref) >= threshold):
                left = x
                break
        right = None
        for x in range(w // 2, w - 2):
            if np.all(np.abs(row[x:x + 3] - ref) >= threshold):
                right = x
                break
        if left is not None and right is not None:
            gap = right - left
            if max(16, int(w * .12)) <= gap <= int(w * .97):
                soil_centers[y] = (float(left + right) / 2.0, y)

    # Use visible berm boundaries whenever available, then fill rows hidden by
    # foliage with the crop-gap center. The previous all-or-nothing preference
    # for green discarded the near-field furrow geometry as soon as a few green
    # pixels appeared in the far field.
    centers = dict(green_centers)
    centers.update(soil_centers)
    if soil_centers and green_centers:
        source = "vegetacion+surcos"
    elif soil_centers:
        source = "surcos"
    elif green_centers:
        source = "vegetacion"
    else:
        source = "sin_borde"
    return [centers[y] for y in sorted(centers)], source
