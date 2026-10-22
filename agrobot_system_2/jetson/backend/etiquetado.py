"""Etiquetado manual de plagas sobre fotos propias (2026-10-22), para entrenar PLAGASX con CAJAS REALES.

El detector actual solo aprendió cajas de imagen completa: no localiza la plaga. Con las fotos del operador (sus coles, su pulgón ceroso)
y cajas dibujadas a mano, el siguiente entrenamiento aprende dónde está el daño. Página: /etiquetar.html.

Fuentes (solo se sirven archivos de estas carpetas; el id es un hash, nunca una ruta):
  usuario   ~/.agrobot/datos_usuario/**   (fotos subidas por el operador)
  capturas  jetson/backend/captures/      (lo que guardó el robot al detectar)
  campo     ~/.agrobot/campo/*/fotos/     (recolección de campo)
Salida: ~/.agrobot/etiquetas/plagas/{images,labels}/<id>.jpg|.txt (YOLO, clases = CLASES, mismas que el notebook PLAGASX) + estado.json.
"""
from __future__ import annotations

import hashlib
import io
import json
import time
import zipfile
from pathlib import Path

from aiohttp import web

# Mismo orden que tools/kaggle/plagasx/plagasx_v2.py: las etiquetas entran directas al entrenamiento.
CLASES = ["col_sana", "col_amarillamiento", "col_manchas_cafe", "col_dano_insecto", "col_mildiu_velloso", "col_malformacion",
          "col_pudricion", "col_mosca_blanca", "col_pulgon", "col_oruga", "col_pulguilla"]
EXT = {".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"}
SALIDA = Path.home() / ".agrobot" / "etiquetas" / "plagas"
LADO_MAX = 1600


def _fuentes(captures_dir: Path) -> dict[str, list[Path]]:
    usuario = Path.home() / ".agrobot" / "datos_usuario"
    campo = Path.home() / ".agrobot" / "campo"
    return {
        "usuario": sorted(p for p in usuario.rglob("*") if p.suffix in EXT) if usuario.exists() else [],
        "capturas": sorted((p for p in captures_dir.glob("*") if p.suffix in EXT), reverse=True) if captures_dir.exists() else [],
        "campo": sorted(p for p in campo.glob("*/fotos/*") if p.suffix in EXT) if campo.exists() else [],
    }


def _id(p: Path) -> str:
    return hashlib.sha1(str(p).encode()).hexdigest()[:16]


class Etiquetado:
    def __init__(self, captures_dir: Path, salida: Path = SALIDA):
        self.captures_dir = Path(captures_dir)
        self.salida = salida
        (salida / "images").mkdir(parents=True, exist_ok=True)
        (salida / "labels").mkdir(parents=True, exist_ok=True)
        self._indice: dict[str, Path] = {}
        self._indice_t = 0.0

    # ---------------- estado ----------------
    def _estado(self) -> dict:
        f = self.salida / "estado.json"
        try:
            return json.loads(f.read_text())
        except (OSError, ValueError):
            return {}

    def _guardar_estado(self, e: dict):
        tmp = self.salida / "estado.json.tmp"
        tmp.write_text(json.dumps(e, ensure_ascii=False, indent=1))
        tmp.replace(self.salida / "estado.json")

    def _ruta(self, ident: str) -> Path | None:
        if ident not in self._indice or time.monotonic() - self._indice_t > 30:
            self._indice = {_id(p): p for lst in _fuentes(self.captures_dir).values() for p in lst}
            self._indice_t = time.monotonic()
        return self._indice.get(ident)

    # ---------------- API ----------------
    async def lista(self, request):
        fuente = request.query.get("fuente", "usuario")
        fs = _fuentes(self.captures_dir).get(fuente, [])[:3000]
        e = self._estado()
        out = []
        for p in fs:
            i = _id(p)
            self._indice[i] = p
            d = e.get(i, {})
            out.append({"id": i, "nombre": p.name, "carpeta": p.parent.name, "revisada": bool(d.get("revisada")),
                        "sin_plaga": bool(d.get("sin_plaga")), "n_cajas": int(d.get("n_cajas", 0))})
        self._indice_t = time.monotonic()
        return web.json_response({"fuente": fuente, "clases": CLASES, "fotos": out})

    async def imagen(self, request):
        p = self._ruta(request.match_info["id"])
        if p is None or not p.exists():
            return web.json_response({"error": "foto desconocida"}, status=404)
        return web.FileResponse(p, headers={"Cache-Control": "max-age=3600"})

    async def leer(self, request):
        ident = request.match_info["id"]
        if self._ruta(ident) is None:
            return web.json_response({"error": "foto desconocida"}, status=404)
        lab = self.salida / "labels" / f"{ident}.txt"
        cajas = []
        if lab.exists():
            for linea in lab.read_text().splitlines():
                p = linea.split()
                if len(p) == 5:
                    cajas.append({"c": int(p[0]), "x": float(p[1]), "y": float(p[2]), "w": float(p[3]), "h": float(p[4])})
        d = self._estado().get(ident, {})
        return web.json_response({"cajas": cajas, "sin_plaga": bool(d.get("sin_plaga")), "revisada": bool(d.get("revisada"))})

    async def guardar(self, request):
        ident = request.match_info["id"]
        p = self._ruta(ident)
        if p is None:
            return web.json_response({"error": "foto desconocida"}, status=404)
        body = await request.json()
        filas = []
        for c in body.get("cajas", []):
            try:
                k, x, y, w, h = int(c["c"]), float(c["x"]), float(c["y"]), float(c["w"]), float(c["h"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (0 <= k < len(CLASES)) or w <= 0.002 or h <= 0.002:
                continue
            x, y = min(max(x, 0.0), 1.0), min(max(y, 0.0), 1.0)
            w, h = min(w, 2 * min(x, 1 - x) + 1e-6), min(h, 2 * min(y, 1 - y) + 1e-6)
            filas.append(f"{k} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
        sin_plaga = bool(body.get("sin_plaga")) and not filas
        (self.salida / "labels" / f"{ident}.txt").write_text("\n".join(filas) + ("\n" if filas else ""))
        destino = self.salida / "images" / f"{ident}.jpg"
        if not destino.exists():
            import cv2                                   # copia reducida: el dataset no guarda fotos de 12 MP
            img = cv2.imread(str(p))
            if img is not None:
                h0, w0 = img.shape[:2]
                k = LADO_MAX / max(h0, w0)
                if k < 1:
                    img = cv2.resize(img, (int(w0 * k), int(h0 * k)), interpolation=cv2.INTER_AREA)
                cv2.imwrite(str(destino), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        e = self._estado()
        e[ident] = {"origen": str(p), "revisada": True, "sin_plaga": sin_plaga, "n_cajas": len(filas), "t": time.strftime("%Y-%m-%d %H:%M:%S")}
        self._guardar_estado(e)
        return web.json_response({"ok": True, "n_cajas": len(filas), "sin_plaga": sin_plaga})

    async def resumen(self, request):
        cuenta = [0] * len(CLASES)
        for lab in (self.salida / "labels").glob("*.txt"):
            for linea in lab.read_text().splitlines():
                p = linea.split()
                if p and p[0].isdigit() and int(p[0]) < len(CLASES):
                    cuenta[int(p[0])] += 1
        e = self._estado()
        return web.json_response({"revisadas": sum(1 for d in e.values() if d.get("revisada")),
                                  "sin_plaga": sum(1 for d in e.values() if d.get("sin_plaga")),
                                  "cajas_por_clase": dict(zip(CLASES, cuenta)), "carpeta": str(self.salida)})

    async def exportar(self, request):
        """Zip listo para subir como dataset de Kaggle: images/, labels/, dataset.yaml (sin particiones: el notebook las hace)."""
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for sub in ("images", "labels"):
                for f in sorted((self.salida / sub).glob("*")):
                    if (self.salida / "labels" / (f.stem + ".txt")).exists():
                        z.write(f, f"{sub}/{f.name}")
            z.writestr("dataset.yaml", "nc: %d\nnames:\n%s" % (len(CLASES), "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASES))))
            z.writestr("LEEME.txt", "Cajas de plaga dibujadas a mano por el operador sobre sus fotos (AGROBOT, %s). Etiqueta vacía = revisada sin plaga.\n"
                       % time.strftime("%Y-%m-%d"))
        return web.Response(body=buf.getvalue(), content_type="application/zip",
                            headers={"Content-Disposition": f'attachment; filename="agrobot_etiquetas_{time.strftime("%Y%m%d_%H%M")}.zip"'})


def install(app, captures_dir):
    e = Etiquetado(captures_dir)
    app.router.add_get("/api/etiquetado/lista", e.lista)
    app.router.add_get("/api/etiquetado/img/{id}", e.imagen)
    app.router.add_get("/api/etiquetado/etiquetas/{id}", e.leer)
    app.router.add_post("/api/etiquetado/etiquetas/{id}", e.guardar)
    app.router.add_get("/api/etiquetado/resumen", e.resumen)
    app.router.add_get("/api/etiquetado/exportar", e.exportar)
    return e
