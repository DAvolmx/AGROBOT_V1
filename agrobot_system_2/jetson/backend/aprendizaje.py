"""
Aprender del operador: correcciones en vivo -> maniobras reutilizables.
======================================================================

La idea, que es lo que el usuario pidió el 2026-09-20: mientras el robot va
en automático, la persona puede tomar el mando en cualquier momento. Esa
intervención NO es solo un pulso: es una lección. Lo que se guarda es

    situación en la que estaba  ->  la SECUENCIA de movimientos que hizo falta

porque salir de un atasco o de un callejón casi nunca es un movimiento
suelto ("atrás, atrás, giro largo a la derecha, adelante").

Cómo se reconoce "la misma situación otra vez": no por coordenadas (el robot
no vuelve nunca al mismo sitio exacto), sino por una FIRMA de lo que percibe
— hueco libre delante/atrás/a los lados en tramos gruesos, dónde cae el
sendero, y qué estaba intentando hacer la autonomía. Dos situaciones con la
misma firma se parecen lo bastante como para que valga la misma maniobra.

No hay red neuronal ni entrenamiento por lotes a propósito: esto tiene que
correr en la Jetson, decidir en milisegundos y, sobre todo, poder EXPLICARSE
("hago esto porque me lo enseñaste aquí"). Es aprendizaje por casos, y los
casos se guardan en JSON legible, listos para revisar a mano.

El mismo fichero lo produce el simulador y lo consume el robot real.
"""
import json
import math
import time
from pathlib import Path

RUTA_POR_DEFECTO = Path.home() / ".agrobot" / "aprendizaje" / "maniobras.json"

# Tramos de distancia libre (m). Gruesos a propósito: si fueran finos, cada
# situación sería única y no se parecería nunca a otra.
TRAMOS_M = (0.35, 0.7, 1.2)
# Dónde cae el centro del sendero respecto del robot.
TRAMO_SENDERO_M = 0.12
# Una intervención termina cuando el operador deja de tocar durante esto.
CIERRE_S = 3.0
# Una maniobra con más fracasos que esto se deja de proponer.
MAX_FRACASOS = 2
# Cuánto se espera, tras aplicar una maniobra, para juzgar si funcionó.
JUICIO_S = 6.0


def _tramo(valor, tramos=TRAMOS_M) -> str:
    """None = nada a la vista en esa dirección = libre del todo."""
    if valor is None:
        return "L"
    for i, t in enumerate(tramos):
        if valor < t:
            return str(i)
    return str(len(tramos))


def firma(sensores: dict, estado: str = "") -> str:
    """Resumen de la situación. Misma firma = misma clase de problema."""
    lat = sensores.get("sendero_lateral_m")
    if lat is None:
        s = "-"
    elif lat > TRAMO_SENDERO_M:
        s = "I"
    elif lat < -TRAMO_SENDERO_M:
        s = "D"
    else:
        s = "C"
    motivo = "libre"
    for clave in ("callejón", "callejon", "bucle", "atasc", "desatasc", "encerrado", "sin salida"):
        if clave in (estado or "").lower():
            motivo = clave.replace("ó", "o")[:7]
            break
    return (f"F{_tramo(sensores.get('lidar_adelante_m'))}"
            f"I{_tramo(sensores.get('lidar_izquierda_m'))}"
            f"D{_tramo(sensores.get('lidar_derecha_m'))}"
            f"A{_tramo(sensores.get('lidar_atras_m'))}"
            f"S{s}|{motivo}")


class Maniobra:
    """Una lección: en esta situación, esta secuencia de movimientos."""

    def __init__(self, firma, pasos, origen="operador", exitos=0, fracasos=0,
                 usos=0, creada=None, nota=""):
        self.firma = firma
        self.pasos = pasos              # [{"accion": ..., "maniobra": bool}, ...]
        self.origen = origen
        self.exitos, self.fracasos, self.usos = exitos, fracasos, usos
        self.creada = creada or time.time()
        self.nota = nota

    @property
    def fiable(self) -> bool:
        return self.fracasos <= MAX_FRACASOS

    @property
    def confianza(self) -> float:
        vistos = self.exitos + self.fracasos
        return 0.5 if not vistos else self.exitos / vistos

    def a_dict(self) -> dict:
        return {"firma": self.firma, "pasos": self.pasos, "origen": self.origen,
                "exitos": self.exitos, "fracasos": self.fracasos, "usos": self.usos,
                "creada": round(self.creada, 1), "nota": self.nota}

    @staticmethod
    def desde_dict(d) -> "Maniobra":
        return Maniobra(d["firma"], d["pasos"], d.get("origen", "operador"),
                        d.get("exitos", 0), d.get("fracasos", 0), d.get("usos", 0),
                        d.get("creada"), d.get("nota", ""))

    def __repr__(self):
        seq = " → ".join(p["accion"] for p in self.pasos)
        return f"<{self.firma}: {seq} ({self.exitos}✓/{self.fracasos}✗)>"


class Aprendizaje:
    """Guarda lo que enseña el operador y lo devuelve cuando hace falta."""

    def __init__(self, ruta: Path | str = RUTA_POR_DEFECTO, autoguardar=True):
        self.ruta = Path(ruta)
        self.autoguardar = autoguardar
        self.maniobras: dict[str, list[Maniobra]] = {}
        self.historial = []            # cada intervención cerrada, para revisar
        self._abierta = None           # intervención en curso
        self._ultimo_toque = 0.0
        self._pendiente = None         # maniobra aplicada, a la espera de juicio
        self.cargar()

    # ------------------------------------------------------------- persistencia
    def cargar(self):
        if not self.ruta.exists():
            return
        try:
            datos = json.loads(self.ruta.read_text())
        except (json.JSONDecodeError, OSError):
            return
        for d in datos.get("maniobras", []):
            m = Maniobra.desde_dict(d)
            self.maniobras.setdefault(m.firma, []).append(m)
        self.historial = datos.get("historial", [])[-500:]

    def guardar(self):
        self.ruta.parent.mkdir(parents=True, exist_ok=True)
        self.ruta.write_text(json.dumps({
            "maniobras": [m.a_dict() for lista in self.maniobras.values() for m in lista],
            "historial": self.historial[-500:],
        }, indent=1, ensure_ascii=False))

    # -------------------------------------------------------------- enseñanza
    def corregir(self, sensores: dict, estado: str, accion: str, maniobra: bool = True,
                 ahora: float | None = None) -> str:
        """El operador tomó el mando. Devuelve la firma de la situación.

        Los pulsos seguidos se agrupan en UNA maniobra: lo que enseña no es
        "gira", es "atrás, atrás, giro largo, adelante"."""
        ahora = time.monotonic() if ahora is None else ahora
        if self._abierta is None or ahora - self._ultimo_toque > CIERRE_S:
            self.cerrar(ahora)
            self._abierta = {"firma": firma(sensores, estado), "estado": estado,
                             "sensores": dict(sensores), "pasos": [], "t": ahora}
        self._abierta["pasos"].append({"accion": accion, "maniobra": bool(maniobra)})
        self._ultimo_toque = ahora
        return self._abierta["firma"]

    def cerrar(self, ahora: float | None = None) -> Maniobra | None:
        """Cierra la intervención en curso y la convierte en maniobra."""
        if not self._abierta or not self._abierta["pasos"]:
            self._abierta = None
            return None
        ahora = time.monotonic() if ahora is None else ahora
        pasos = [p for p in self._abierta["pasos"] if p["accion"] != "parar"]
        if not pasos:
            self._abierta = None
            return None
        m = Maniobra(self._abierta["firma"], pasos)
        lista = self.maniobras.setdefault(m.firma, [])
        igual = next((x for x in lista if [p["accion"] for p in x.pasos]
                      == [p["accion"] for p in pasos]), None)
        if igual is not None:           # ya me lo habías enseñado: cuenta como refuerzo
            igual.exitos += 1
            m = igual
        else:
            lista.append(m)
        self.historial.append({"t": round(time.time()), "firma": m.firma,
                               "pasos": [p["accion"] for p in pasos],
                               "estado": self._abierta["estado"]})
        self._abierta = None
        if self.autoguardar:
            self.guardar()
        return m

    # -------------------------------------------------------------- uso
    def consultar(self, sensores: dict, estado: str = "") -> Maniobra | None:
        """¿Me enseñaron algo para esta situación?"""
        candidatas = [m for m in self.maniobras.get(firma(sensores, estado), []) if m.fiable]
        if not candidatas:
            return None
        return max(candidatas, key=lambda m: (m.confianza, m.exitos))

    def aplicada(self, m: Maniobra, sensores: dict, ahora: float | None = None):
        """Se acaba de ejecutar una maniobra aprendida: queda a prueba."""
        ahora = time.monotonic() if ahora is None else ahora
        m.usos += 1
        self._pendiente = {"maniobra": m, "t": ahora, "sensores": dict(sensores)}

    def juzgar(self, sensores: dict, pose_movida_m: float, ahora: float | None = None):
        """¿Funcionó lo aplicado? Se mira si el robot se despejó y se movió.

        Sin esto las maniobras malas se propondrían para siempre."""
        if not self._pendiente:
            return None
        ahora = time.monotonic() if ahora is None else ahora
        if ahora - self._pendiente["t"] < JUICIO_S:
            return None
        m = self._pendiente["maniobra"]
        antes = self._pendiente["sensores"].get("lidar_adelante_m")
        ahora_f = sensores.get("lidar_adelante_m")
        despejo = (ahora_f is None) or (antes is not None and ahora_f > antes + 0.15)
        ok = bool(despejo or pose_movida_m > 0.25)
        (m.__dict__.__setitem__("exitos", m.exitos + 1) if ok
         else m.__dict__.__setitem__("fracasos", m.fracasos + 1))
        self._pendiente = None
        if self.autoguardar:
            self.guardar()
        return ok

    # -------------------------------------------------------------- resumen
    def resumen(self) -> dict:
        todas = [m for lista in self.maniobras.values() for m in lista]
        return {
            "situaciones": len(self.maniobras),
            "maniobras": len(todas),
            "fiables": sum(1 for m in todas if m.fiable),
            "usos": sum(m.usos for m in todas),
            "exitos": sum(m.exitos for m in todas),
            "fracasos": sum(m.fracasos for m in todas),
            "intervenciones": len(self.historial),
            "ruta": str(self.ruta),
            "aprendidas": [
                {"firma": m.firma, "pasos": [p["accion"] for p in m.pasos],
                 "exitos": m.exitos, "fracasos": m.fracasos, "usos": m.usos,
                 "confianza": round(m.confianza, 2)}
                for m in sorted(todas, key=lambda m: -m.usos)[:40]],
        }

    def olvidar(self, firma_borrar: str) -> int:
        n = len(self.maniobras.pop(firma_borrar, []))
        if n and self.autoguardar:
            self.guardar()
        return n
