"""¿Qué planta hay en la maceta? Clasificación de VOCABULARIO ABIERTO con CLIP (ViT-B/32), sin entrenar (2026-10-23).

Ninguno de los modelos propios distingue una col de otra planta en maceta: el detector de macetas solo ve macetas y el clasificador PLAGASX
solo conoce enfermedades de col (a cualquier planta le pone una). CLIP compara el recorte con frases; con la cámara del robot y tres macetas
(col marchita, planta seca, planta con flores) dio col 0,75 frente a 0,03 y 0,01. ~0,5 s en CPU: se usa una vez por inspección.
"""
from __future__ import annotations

import threading

# Frases por grupo (se suman las probabilidades del grupo). Descriptivas = mucho mejor en la cámara del robot de noche: col 0,57-0,64
# frente a ≤0,27 en las otras dos macetas, contra 0,35-0,47 / ≤0,11 con frases genéricas (2026-10-23).
GRUPOS = {
    "col": ["a wilted cauliflower plant with large leaves in a pot", "a cabbage plant with big broad leaves in a black pot",
            "a brassica vegetable plant in a pot"],
    "otra": ["a thin plant with small pink flowers in a black pot", "a small dry twig in a black pot", "a bare stem plant in a pot",
             "a houseplant with small leaves in a pot"],
}
FRASES = {f"{g}_{i}": t for g, ts in GRUPOS.items() for i, t in enumerate(ts)}
_modelo = None
_lock = threading.Lock()


def cargar():
    """Carga perezosa (y precarga en segundo plano al arrancar: la primera vez tarda ~10-60 s)."""
    global _modelo
    with _lock:
        if _modelo is None:
            import clip
            import torch
            m, prep = clip.load("ViT-B/32", device="cpu")
            with torch.no_grad():
                t = m.encode_text(clip.tokenize(list(FRASES.values())))
                t /= t.norm(dim=-1, keepdim=True)
            _modelo = (m, prep, t)
    return _modelo


def precargar():
    threading.Thread(target=lambda: _seguro(cargar), daemon=True).start()


def _seguro(fn):
    try:
        fn()
    except Exception as exc:
        print(f"[especie] CLIP no disponible: {exc}", flush=True)


def clasificar(frame_bgr, xyxy=None) -> dict:
    """{'especie': clave más probable, 'p_col': prob. de col, 'probs': {...}} para el recorte (o la imagen entera)."""
    import cv2
    import torch
    from PIL import Image
    m, prep, t = cargar()
    img = frame_bgr
    if xyxy is not None:                       # recorte JUSTO a la caja: con margen entra el fondo y la col bajaba de 0,41 a 0,11
        h, w = frame_bgr.shape[:2]
        x1, y1, x2, y2 = (int(v) for v in xyxy)
        img = frame_bgr[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
    x = prep(Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))).unsqueeze(0)
    with torch.no_grad():
        f = m.encode_image(x)
        f /= f.norm(dim=-1, keepdim=True)
        p = (100 * f @ t.T).softmax(-1)[0].tolist()
    probs = dict(zip(FRASES, p))
    p_col = sum(v for k, v in probs.items() if k.startswith("col_"))
    return {"especie": "col" if p_col >= 0.5 else "otra", "p_col": round(p_col, 3), "es_col": p_col >= 0.5,
            "frase_top": FRASES[max(probs, key=probs.get)]}
