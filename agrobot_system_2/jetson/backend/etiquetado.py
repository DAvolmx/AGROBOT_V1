"""Etiquetado manual de plagas sobre fotos propias (2026-10-22), para entrenar PLAGASX con CAJAS REALES.

El detector actual solo aprendió cajas de imagen completa: no localiza la plaga. Con las fotos del operador (sus coles, su pulgón ceroso)
y cajas dibujadas a mano, el siguiente entrenamiento aprende dónde está el daño. Página: /etiquetar.html.

Fuentes (solo se sirven archivos de estas carpetas; el id es un hash, nunca una ruta):
  usuario   ~/.agrobot/datos_usuario/**   (fotos subidas por el operador)
  capturas  jetson/backend/captures/      (lo que guardó el robot al detectar)
  campo     ~/.agrobot/campo/*/fotos/     (recolección de campo)
  mision    ~/.agrobot/dataset_plantas/<sesión>/  (misión «Recoger dataset de plantas»; se ven según las va tomando)
Salida: ~/.agrobot/etiquetas/plagas/{images,labels}/<id>.jpg|.txt (YOLO) + estado.json + clases.json.

Etiquetas sin límite (2026-11-08): clases.json parte de CLASES y solo crece. El número de una clase es su posición, así que nunca se borra ni se
reordena (cambiaría el significado de las cajas ya guardadas); una etiqueta mal escrita se renombra.
"""
from __future__ import annotations

import asyncio
import hashlib
import json
import time
import zipfile
from pathlib import Path

from aiohttp import web

# Mismo orden que tools/kaggle/plagasx/plagasx_v2.py: las etiquetas entran directas al entrenamiento.
CLASES = ["col_sana", "col_amarillamiento", "col_manchas_cafe", "col_dano_insecto", "col_mildiu_velloso", "col_malformacion",
          "col_pudricion", "col_mosca_blanca", "col_pulgon", "col_oruga", "col_pulguilla", "col_trips"]
EXT = {".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"}
SALIDA = Path.home() / ".agrobot" / "etiquetas" / "plagas"
LADO_MAX = 1600
MAX_SUBIDA = 8 << 30
MAX_DESCOMPRIMIDO = 30 << 30
MAX_ARCHIVOS = 200_000


def _fuentes(captures_dir: Path) -> dict[str, list[Path]]:
    usuario = Path.home() / ".agrobot" / "datos_usuario"
    campo = Path.home() / ".agrobot" / "campo"
    mision = Path.home() / ".agrobot" / "dataset_plantas"
    return {
        "mision": sorted(p for p in mision.glob("*/*") if p.suffix in EXT) if mision.exists() else [],    # en orden de toma: las nuevas van al final
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

    # ---------------- clases ----------------
    def clases(self) -> list[str]:
        try:
            guardadas = json.loads((self.salida / "clases.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            guardadas = []
        return guardadas if len(guardadas) >= len(CLASES) else CLASES + guardadas[len(CLASES):]

    def _guardar_clases(self, clases: list[str]):
        tmp = self.salida / "clases.json.tmp"
        tmp.write_text(json.dumps(clases, ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(self.salida / "clases.json")

    @staticmethod
    def _nombre_valido(nombre) -> str | None:
        """Una sola palabra para YOLO y dataset.yaml: «Hoja Rizada» -> «hoja_rizada»."""
        n = "_".join(str(nombre or "").strip().lower().split()).strip("_")
        return n if n and len(n) <= 60 and all(c.isalnum() or c in "_-" for c in n) else None

    async def agregar_clase(self, request):
        body = await request.json()
        n = self._nombre_valido(body.get("nombre"))
        if n is None:
            return web.json_response({"error": "Nombre no válido: usa letras, números, - o _ (hasta 60)."}, status=400)
        clases = self.clases()
        if n in clases:
            return web.json_response({"clases": clases, "indice": clases.index(n), "ya_existia": True})
        clases.append(n)
        self._guardar_clases(clases)
        return web.json_response({"clases": clases, "indice": len(clases) - 1, "ya_existia": False})

    async def renombrar_clase(self, request):
        body = await request.json()
        n = self._nombre_valido(body.get("nombre"))
        clases = self.clases()
        try:
            i = int(body.get("indice"))
        except (TypeError, ValueError):
            i = -1
        if n is None or not 0 <= i < len(clases):
            return web.json_response({"error": "Nombre o etiqueta no válidos."}, status=400)
        if n in clases and clases.index(n) != i:
            return web.json_response({"error": f"Ya existe la etiqueta {n}."}, status=409)
        clases[i] = n
        self._guardar_clases(clases)
        return web.json_response({"clases": clases})

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
        fs = _fuentes(self.captures_dir).get(fuente, [])
        e = self._estado()
        out = []
        for p in fs:
            i = _id(p)
            self._indice[i] = p
            d = e.get(i, {})
            out.append({"id": i, "nombre": p.name, "carpeta": p.parent.name, "revisada": bool(d.get("revisada")),
                        "sin_plaga": bool(d.get("sin_plaga")), "n_cajas": int(d.get("n_cajas", 0))})
        self._indice_t = time.monotonic()
        return web.json_response({"fuente": fuente, "clases": self.clases(), "fotos": out})

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
        filas, n_clases = [], len(self.clases())
        for c in body.get("cajas", []):
            try:
                k, x, y, w, h = int(c["c"]), float(c["x"]), float(c["y"]), float(c["w"]), float(c["h"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (0 <= k < n_clases) or w <= 0.002 or h <= 0.002:
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

    # ---------------- subir datasets (2026-11-09) ----------------
    @staticmethod
    def _nombres_zip(ruta: Path) -> list[str] | None:
        """Nombres de clase declarados en un zip YOLO (data.yaml, dataset.yaml, classes.txt u obj.names); None si no declara ninguno."""
        import yaml
        with zipfile.ZipFile(ruta) as z:
            for info in sorted(z.infolist(), key=lambda i: i.filename.count("/")):
                base = info.filename.rsplit("/", 1)[-1].lower()
                if info.file_size > 1_000_000 or base not in {"data.yaml", "dataset.yaml", "data.yml", "classes.txt", "obj.names"}:
                    continue
                texto = z.read(info).decode("utf-8", "ignore")
                if base.endswith((".txt", ".names")):
                    return [l.strip() for l in texto.splitlines() if l.strip()]
                try:
                    n = (yaml.safe_load(texto) or {}).get("names")
                except yaml.YAMLError:
                    continue
                if isinstance(n, dict):
                    return [str(n[k]) for k in sorted(n, key=int)]
                if isinstance(n, list):
                    return [str(x) for x in n]
        return None

    def _guardar_subida(self, datos: bytes, clave: str, carpeta: Path, filas: list[str] | None, entradas: dict) -> bool:
        import cv2
        import numpy as np
        img = cv2.imdecode(np.frombuffer(datos, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return False
        h0, w0 = img.shape[:2]
        k = LADO_MAX / max(h0, w0)
        if k < 1:
            img = cv2.resize(img, (int(w0 * k), int(h0 * k)), interpolation=cv2.INTER_AREA)
        destino = carpeta / f"{hashlib.sha1(clave.encode()).hexdigest()[:16]}.jpg"            # el nombre nunca sale de la ruta del zip
        cv2.imwrite(str(destino), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if filas is not None:
            ident = _id(destino)
            (self.salida / "labels" / f"{ident}.txt").write_text("".join(f + "\n" for f in filas))
            cv2.imwrite(str(self.salida / "images" / f"{ident}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            entradas[ident] = {"origen": str(destino), "revisada": True, "sin_plaga": not filas, "n_cajas": len(filas), "t": time.strftime("%Y-%m-%d %H:%M:%S")}
        return True

    def _importar(self, recibidos: list[tuple[Path, str]], carpeta: Path, mapas: dict) -> tuple[dict, dict]:
        """Corre en un hilo. Acepta fotos sueltas y zips (con o sin cajas YOLO); ignora todo lo demás y nunca escribe rutas tomadas del zip."""
        carpeta.mkdir(parents=True, exist_ok=True)
        entradas: dict = {}
        r = {"fotos": 0, "con_etiqueta": 0, "cajas": 0, "ignorados": 0, "sin_nombres_de_clase": 0}
        for ruta, nombre in recibidos:
            if not nombre.lower().endswith(".zip"):
                if Path(nombre).suffix in EXT and self._guardar_subida(ruta.read_bytes(), nombre, carpeta, None, entradas):
                    r["fotos"] += 1
                else:
                    r["ignorados"] += 1
                continue
            mapa = mapas.get(str(ruta))
            with zipfile.ZipFile(ruta) as z:
                infos = [i for i in z.infolist() if not i.is_dir() and "__MACOSX" not in i.filename]
                if len(infos) > MAX_ARCHIVOS or sum(i.file_size for i in infos) > MAX_DESCOMPRIMIDO:
                    raise ValueError("El zip es demasiado grande al descomprimirlo.")
                textos = {i.filename: i for i in infos if i.filename.lower().endswith(".txt") and i.file_size < 5_000_000}
                for i in infos:
                    if Path(i.filename).suffix not in EXT:
                        continue
                    base = i.filename.rsplit(".", 1)[0]
                    par = ("/" + base).replace("/images/", "/labels/", 1).lstrip("/") + ".txt"
                    lab = textos.get(par) or textos.get(base + ".txt")
                    filas = None
                    if lab is not None and mapa is None:
                        r["sin_nombres_de_clase"] += 1
                    elif lab is not None:
                        filas = []
                        for linea in z.read(lab).decode("utf-8", "ignore").splitlines():
                            p = linea.split()
                            try:
                                if len(p) == 5 and int(p[0]) in mapa and all(0 <= float(v) <= 1 for v in p[1:]):
                                    filas.append(f"{mapa[int(p[0])]} {' '.join(p[1:])}")
                            except ValueError:
                                pass
                    if self._guardar_subida(z.read(i), f"{nombre}:{i.filename}", carpeta, filas, entradas):
                        r["fotos"] += 1
                        if filas is not None:
                            r["con_etiqueta"] += 1; r["cajas"] += len(filas)
                    else:
                        r["ignorados"] += 1
        return entradas, r

    async def subir(self, request):
        """POST multipart: fotos sueltas o zips. Con cajas YOLO y nombres de clase, entran ya etiquetadas (las etiquetas nuevas se añaden sin límite); sin ellas, quedan para etiquetar."""
        tmp = self.salida / "subidas_tmp"
        tmp.mkdir(exist_ok=True)
        recibidos: list[tuple[Path, str]] = []
        nombre_ds, total = "dataset", 0
        try:
            lector = await request.multipart()
            while (parte := await lector.next()) is not None:
                if not parte.filename:
                    if parte.name == "nombre":
                        nombre_ds = self._nombre_valido(await parte.text()) or "dataset"
                    continue
                ruta = tmp / f"{time.time_ns()}_{len(recibidos)}"
                recibidos.append((ruta, Path(parte.filename).name))
                with ruta.open("wb") as f:
                    while trozo := await parte.read_chunk(1 << 20):
                        total += len(trozo)
                        if total > MAX_SUBIDA:
                            return web.json_response({"error": "La subida supera el límite de 8 GB."}, status=413)
                        f.write(trozo)
            if not recibidos:
                return web.json_response({"error": "No llegó ningún archivo."}, status=400)
            nombres = {}
            for ruta, nom in recibidos:
                if nom.lower().endswith(".zip"):
                    try:
                        nombres[str(ruta)] = await asyncio.to_thread(self._nombres_zip, ruta)
                    except zipfile.BadZipFile:
                        return web.json_response({"error": f"{nom} no es un zip válido."}, status=400)
            # Registro de clases y estado en el bucle, sin await de por medio: un guardado simultáneo no puede pisarlos.
            clases, mapas = self.clases(), {}
            for r, ns in nombres.items():
                if ns is None:
                    continue
                mapa = {}
                for i, n in enumerate(ns):
                    v = self._nombre_valido(n)
                    if v is None:
                        continue
                    if v not in clases:
                        clases.append(v)
                    mapa[i] = clases.index(v)
                mapas[r] = mapa
            self._guardar_clases(clases)
            carpeta = Path.home() / ".agrobot" / "datos_usuario" / "subidas" / f"{time.strftime('%Y%m%d_%H%M%S')}_{nombre_ds}"
            try:
                entradas, res = await asyncio.to_thread(self._importar, recibidos, carpeta, mapas)
            except ValueError as ex:
                return web.json_response({"error": str(ex)}, status=413)
            e = self._estado(); e.update(entradas); self._guardar_estado(e)
            self._indice_t = 0.0
            return web.json_response({"ok": True, "carpeta": carpeta.name, "clases": len(clases), **res})
        finally:
            for ruta, _ in recibidos:
                ruta.unlink(missing_ok=True)

    async def resumen(self, request):
        clases = self.clases()
        cuenta = [0] * len(clases)
        for lab in (self.salida / "labels").glob("*.txt"):
            for linea in lab.read_text().splitlines():
                p = linea.split()
                if p and p[0].isdigit() and int(p[0]) < len(clases):
                    cuenta[int(p[0])] += 1
        e = self._estado()
        return web.json_response({"revisadas": sum(1 for d in e.values() if d.get("revisada")),
                                  "sin_plaga": sum(1 for d in e.values() if d.get("sin_plaga")),
                                  "cajas_por_clase": dict(zip(clases, cuenta)), "carpeta": str(self.salida)})

    async def exportar(self, request):
        """Zip listo para subir como dataset de Kaggle: images/, labels/, dataset.yaml (sin particiones: el notebook las hace)."""
        # En un hilo y a disco: con un dataset grande, comprimir en el bucle del backend frenaría las órdenes a la C3.
        zip_ruta = self.salida / "ultima_exportacion.zip"              # se sobrescribe: no se acumulan copias
        await asyncio.to_thread(self._escribir_zip, zip_ruta, self.clases())
        nombre = f"agrobot_etiquetas_{time.strftime('%Y%m%d_%H%M')}.zip"
        return web.FileResponse(zip_ruta, headers={"Content-Type": "application/zip", "Content-Disposition": f'attachment; filename="{nombre}"'})

    def _escribir_zip(self, destino: Path, clases: list[str]):
        with zipfile.ZipFile(destino, "w", zipfile.ZIP_DEFLATED) as z:
            for sub in ("images", "labels"):
                for f in sorted((self.salida / sub).glob("*")):
                    if (self.salida / "labels" / (f.stem + ".txt")).exists():
                        z.write(f, f"{sub}/{f.name}")
            z.writestr("dataset.yaml", "nc: %d\nnames:\n%s" % (len(clases), "".join(f"  {i}: {n}\n" for i, n in enumerate(clases))))
            z.writestr("LEEME.txt", "Cajas de plaga dibujadas a mano por el operador sobre sus fotos (AGROBOT, %s). Etiqueta vacía = revisada sin plaga.\n"
                       % time.strftime("%Y-%m-%d"))


def install(app, captures_dir):
    e = Etiquetado(captures_dir)
    app.router.add_get("/api/etiquetado/lista", e.lista)
    app.router.add_get("/api/etiquetado/img/{id}", e.imagen)
    app.router.add_get("/api/etiquetado/etiquetas/{id}", e.leer)
    app.router.add_post("/api/etiquetado/etiquetas/{id}", e.guardar)
    app.router.add_get("/api/etiquetado/resumen", e.resumen)
    app.router.add_post("/api/etiquetado/clases", e.agregar_clase)
    app.router.add_post("/api/etiquetado/clases/renombrar", e.renombrar_clase)
    app.router.add_get("/api/etiquetado/exportar", e.exportar)
    app.router.add_post("/api/etiquetado/subir", e.subir)
    return e
