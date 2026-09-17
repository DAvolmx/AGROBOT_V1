"""
Mapas con nombre: guardar la sesión de SLAM y reusarla otro día.
===============================================================

Cada reinicio de ROS empezaba un mapa nuevo con el origen donde estuviera el
robot, y el recorrido de patrulla grabado dejaba de valer: sus puntos eran
coordenadas del mapa anterior. Esto guarda las dos cosas juntas bajo un
nombre y las devuelve juntas.

Se guarda el GRAFO de poses de slam_toolbox (`serialize_map`), no solo la
imagen. Al cargarlo (`deserialize_map`) slam_toolbox sigue en modo mapeo
sobre ese grafo: el mapa no queda congelado, sigue creciendo y corrigiéndose
mientras el robot patrulla.

⚠️ Al cargar se usa START_AT_FIRST_NODE: slam_toolbox asume que el robot está
donde empezó ese mapa. El robot tiene que estar en esa BASE al cargarlo; si
no, el mapa queda desplazado respecto de la realidad.

Estructura en disco (una carpeta por mapa):
    maps/<nombre>/mapa.posegraph   grafo de slam_toolbox
    maps/<nombre>/mapa.data        scans asociados al grafo
    maps/<nombre>/mapa.pgm|.yaml   imagen, para verlo o usarlo con map_server
    maps/<nombre>/waypoints.json   recorrido de patrulla de este mapa
    maps/<nombre>/info.json        fecha y resumen
"""
import asyncio
import json
import re
import shutil
import subprocess
import time
from pathlib import Path

# Letras (con acentos), números, guion y guion bajo. Nada de "/" ni "..":
# el nombre acaba siendo una ruta en disco.
_NOMBRE_VALIDO = re.compile(r"[\w\-]{1,40}")


class ErrorMapa(Exception):
    """Fallo esperable que se le muestra tal cual al operador."""


def validar_nombre(nombre: str) -> str:
    nombre = (nombre or "").strip().replace(" ", "_")
    if not _NOMBRE_VALIDO.fullmatch(nombre) or nombre.startswith("_"):
        raise ErrorMapa("nombre inválido: usá letras, números, - o _ (hasta 40)")
    return nombre


def _servicio(servicio: str, tipo: str, args: str, timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(["ros2", "service", "call", servicio, tipo, args],
                          capture_output=True, text=True, timeout=timeout)


class GestorMapas:
    def __init__(self, carpeta: Path):
        self.carpeta = carpeta
        self.carpeta.mkdir(parents=True, exist_ok=True)

    def listar(self) -> list[dict]:
        mapas = []
        for d in self.carpeta.iterdir():
            if d.name.startswith("_") or not (d / "mapa.posegraph").exists():
                continue            # "_guardando_*": un guardado en curso
            try:
                info = json.loads((d / "info.json").read_text())
            except (OSError, json.JSONDecodeError):
                info = {}
            peso = sum(f.stat().st_size for f in d.iterdir() if f.is_file())
            mapas.append({
                "nombre": d.name,
                "guardado": info.get("guardado", (d / "mapa.posegraph").stat().st_mtime),
                "puntos": info.get("puntos", 0),
                "peso_kb": round(peso / 1024),
            })
        return sorted(mapas, key=lambda m: m["guardado"], reverse=True)

    def existe(self, nombre: str) -> bool:
        return (self.carpeta / validar_nombre(nombre) / "mapa.posegraph").exists()

    async def guardar(self, nombre: str, waypoints: list[dict], sobrescribir: bool) -> dict:
        nombre = validar_nombre(nombre)
        destino = self.carpeta / nombre
        if (destino / "mapa.posegraph").exists() and not sobrescribir:
            raise ErrorMapa(f"ya existe un mapa llamado «{nombre}»")

        # Se escribe en una carpeta temporal y se cambia de sitio al final:
        # si slam_toolbox falla a mitad, el mapa anterior con ese nombre no
        # queda a medio pisar.
        tmp = self.carpeta / f"_guardando_{nombre}"
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir()
        base = str(tmp / "mapa")
        try:
            res = await asyncio.to_thread(
                _servicio, "/slam_toolbox/serialize_map",
                "slam_toolbox/srv/SerializePoseGraph", f"{{filename: '{base}'}}", 60)
            if res.returncode != 0 or not (tmp / "mapa.posegraph").exists():
                raise ErrorMapa("slam_toolbox no pudo guardar el grafo — ¿está corriendo el SLAM?")

            # La imagen es un extra: si falla, el mapa sigue siendo reusable.
            await asyncio.to_thread(
                _servicio, "/slam_toolbox/save_map", "slam_toolbox/srv/SaveMap",
                f"{{name: {{data: '{base}'}}}}", 30)

            (tmp / "waypoints.json").write_text(json.dumps(waypoints, indent=2))
            info = {"nombre": nombre, "guardado": time.time(), "puntos": len(waypoints)}
            (tmp / "info.json").write_text(json.dumps(info, indent=2))

            shutil.rmtree(destino, ignore_errors=True)
            tmp.rename(destino)
            return info
        except subprocess.TimeoutExpired:
            raise ErrorMapa("slam_toolbox tardó demasiado en guardar")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    async def cargar(self, nombre: str) -> list[dict]:
        """Carga el grafo en slam_toolbox y devuelve el recorrido del mapa."""
        nombre = validar_nombre(nombre)
        origen = self.carpeta / nombre
        if not (origen / "mapa.posegraph").exists():
            raise ErrorMapa(f"no existe el mapa «{nombre}»")

        try:
            res = await asyncio.to_thread(
                _servicio, "/slam_toolbox/deserialize_map",
                "slam_toolbox/srv/DeserializePoseGraph",
                f"{{filename: '{origen / 'mapa'}', match_type: 1}}", 60)
        except subprocess.TimeoutExpired:
            raise ErrorMapa("slam_toolbox tardó demasiado en cargar el mapa")
        if res.returncode != 0:
            raise ErrorMapa("slam_toolbox no pudo cargar el mapa — ¿está corriendo el SLAM?")

        try:
            return json.loads((origen / "waypoints.json").read_text())
        except (OSError, json.JSONDecodeError):
            return []
