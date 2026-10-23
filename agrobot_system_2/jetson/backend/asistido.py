"""Manejo asistido: el operador toma el mando MIENTRAS el robot está en modo autónomo, y cada corrección queda
registrada para mejorar al robot (sustituye al antiguo dashboard de entrenamiento).

Qué hace:
  · `tomar()`   — la autonomía queda pausada (el servidor la detiene) y se recuerda qué estaba haciendo y qué
                  ordenaba en ese instante (`auto_cmd`).
  · `muestra()` — cada orden manual se compara con la de la autonomía, se clasifica y se guarda en JSONL con la
                  pose y las distancias del LIDAR (máx. 5 por segundo).
  · `sugerencias()` — con varias correcciones del mismo tipo propone ajustes (margen de parada, cono lateral,
                  velocidad). NUNCA los aplica solo: el operador pulsa «Aplicar».

Sin dependencias del servidor: se prueba en tests/test_asistido.py.
"""
from __future__ import annotations

import json
import time
from collections import Counter
from pathlib import Path

PERIODO_MUESTRA_S = 0.2          # máx. 5 muestras por segundo
UMBRAL_V = 0.02                  # m/s por debajo de lo cual se considera «parado»
UMBRAL_W = 0.05                  # rad/s por debajo de lo cual se considera «sin girar»
MIN_EVENTOS = 4                  # correcciones del mismo tipo para sugerir algo
CADUCA_S = 300.0                 # sin órdenes manuales durante tanto, la intervención se da por terminada

TIPOS = {
    "freno": "Frenaste a la autonomía cuando avanzaba",
    "empuje": "Avanzaste cuando la autonomía estaba parada o bloqueada",
    "giro_contrario": "Giraste al lado contrario del que giraba",
    "giro_extra": "Giraste cuando la autonomía iba recta",
    "giro_mismo": "Giraste igual que la autonomía (la reforzaste)",
    "avance": "Avanzaste en la misma dirección",
    "pose": "Corregiste la pose en el mapa",
}


def clasificar(auto_v: float, auto_w: float, v: float, w: float) -> str:
    """Compara la orden del operador (v, w) con la que daba la autonomía."""
    if abs(w) > UMBRAL_W:
        if abs(auto_w) <= UMBRAL_W:
            return "giro_extra"
        return "giro_contrario" if auto_w * w < 0 else "giro_mismo"
    if auto_v > UMBRAL_V and v <= UMBRAL_V:
        return "freno"
    if auto_v <= UMBRAL_V and v > UMBRAL_V:
        return "empuje"
    return "avance"


class Asistido:
    def __init__(self, carpeta: Path, reloj=time.time):
        self.carpeta = Path(carpeta)
        self.reloj = reloj
        self.activa = False                 # ¿el operador tiene el mando tras interrumpir a la autonomía?
        self.modo_previo: str | None = None
        self.meta_previa: dict | None = None
        self.auto_cmd = (0.0, 0.0)
        self.desde = 0.0
        self.eventos_sesion = 0
        self._ultima = 0.0
        self.ultima_orden = 0.0             # última orden manual (aunque la muestra se descarte por frecuencia)
        self.recientes: list[dict] = []     # las últimas muestras (para el panel)

    # ---- ciclo de vida ----
    def tomar(self, modo: str, auto_cmd=(0.0, 0.0), meta: dict | None = None):
        if self.activa:
            return
        self.activa, self.modo_previo, self.meta_previa = True, modo, meta
        self.auto_cmd = (float(auto_cmd[0]), float(auto_cmd[1]))
        self.desde = self.reloj()
        self.eventos_sesion = 0
        self._ultima = 0.0
        self.ultima_orden = self.desde

    def soltar(self) -> str | None:
        modo = self.modo_previo
        self.activa = False
        self.modo_previo = None
        return modo

    def _caducar(self, ahora: float):
        """Una intervención olvidada (nadie reanudó ni soltó) no debe seguir registrando órdenes manuales horas después."""
        if self.activa and ahora - max(self._ultima, self.ultima_orden, self.desde) > CADUCA_S:
            self.soltar()

    # ---- registro ----
    def muestra(self, v: float, w: float, pose: dict | None, dist: dict | None, origen: str = "manual") -> dict | None:
        """Registra una orden manual dada durante la intervención. None si se descarta por el límite de frecuencia."""
        ahora = self.reloj()
        self._caducar(ahora)
        if self.activa:
            self.ultima_orden = ahora
        if not self.activa or ahora - self._ultima < PERIODO_MUESTRA_S:
            return None
        self._ultima = ahora
        tipo = clasificar(self.auto_cmd[0], self.auto_cmd[1], float(v), float(w))
        ev = {"t": round(ahora, 2), "tipo": tipo, "modo": self.modo_previo, "origen": origen,
              "auto_v": round(self.auto_cmd[0], 3), "auto_w": round(self.auto_cmd[1], 3),
              "v": round(float(v), 3), "w": round(float(w), 3),
              "pose": {k: round(float(pose[k]), 3) for k in ("x", "y", "theta_deg")} if pose else None,
              "dist_cm": {k: (None if x is None else round(float(x) * 100)) for k, x in (dist or {}).items()}}
        self._guardar(ev)
        self.eventos_sesion += 1
        self.recientes = (self.recientes + [ev])[-12:]
        return ev

    def evento_pose(self, x: float, y: float, theta: float, pose_antes: dict | None):
        """Corrección de pose en el mapa (siempre se registra, haya o no intervención)."""
        ev = {"t": round(self.reloj(), 2), "tipo": "pose", "modo": self.modo_previo, "origen": "mapa",
              "pose": {"x": round(x, 3), "y": round(y, 3), "theta_deg": round(theta, 1)},
              "pose_antes": pose_antes}
        self._guardar(ev)
        return ev

    def _guardar(self, ev: dict):
        try:
            self.carpeta.mkdir(parents=True, exist_ok=True)
            f = self.carpeta / (time.strftime("%Y-%m-%d", time.localtime(ev["t"])) + ".jsonl")
            with f.open("a") as fh:
                fh.write(json.dumps(ev, ensure_ascii=False) + "\n")
        except OSError:
            pass                              # el registro nunca debe tumbar el manejo

    # ---- lectura ----
    def leer(self, max_archivos: int = 14) -> list[dict]:
        out = []
        for f in sorted(self.carpeta.glob("*.jsonl"))[-max_archivos:]:
            for linea in f.read_text().splitlines():
                try:
                    out.append(json.loads(linea))
                except ValueError:
                    continue
        return out

    def resumen(self, eventos: list[dict] | None = None) -> dict:
        ev = self.leer() if eventos is None else eventos
        cont = Counter(e["tipo"] for e in ev)
        return {"total": len(ev), "por_tipo": {t: {"n": cont.get(t, 0), "texto": txt} for t, txt in TIPOS.items() if cont.get(t, 0)},
                "por_modo": dict(Counter(e.get("modo") or "—" for e in ev))}

    def sugerencias(self, cfg: dict, eventos: list[dict] | None = None) -> list[dict]:
        """Ajustes propuestos a partir de las correcciones. `cfg` = la config del backend (secciones slam/autonomia)."""
        ev = self.leer() if eventos is None else eventos
        slam, auto = cfg.get("slam", {}), cfg.get("autonomia", {})
        sug: list[dict] = []

        def frontal_cm(e):
            return (e.get("dist_cm") or {}).get("adelante")

        frenos = [e for e in ev if e["tipo"] == "freno"]
        cerca = [e for e in frenos if frontal_cm(e) is not None and frontal_cm(e) < 70]
        lejos = [e for e in frenos if frontal_cm(e) is not None and frontal_cm(e) >= 70]
        if len(cerca) >= MIN_EVENTOS:
            actual = int(slam.get("margen_parada_cm", 26))
            sug.append({"id": "margen_mas", "seccion": "slam", "clave": "margen_parada_cm", "actual": actual,
                        "propuesto": min(60, actual + 5), "eventos": len(cerca),
                        "texto": f"Frenaste {len(cerca)} veces con algo a menos de 70 cm: la autonomía debería parar antes."})
        if len(lejos) >= MIN_EVENTOS:
            actual = float(auto.get("escala_velocidad", 0.5))
            sug.append({"id": "velocidad_menos", "seccion": "autonomia", "clave": "escala_velocidad", "actual": actual,
                        "propuesto": round(max(0.1, actual - 0.05), 2), "eventos": len(lejos),
                        "texto": f"Frenaste {len(lejos)} veces con el camino libre: la autonomía va demasiado rápido."})
        empujes = [e for e in ev if e["tipo"] == "empuje" and frontal_cm(e) is not None and frontal_cm(e) > 40]
        if len(empujes) >= MIN_EVENTOS:
            actual = int(slam.get("margen_parada_cm", 26))
            sug.append({"id": "margen_menos", "seccion": "slam", "clave": "margen_parada_cm", "actual": actual,
                        "propuesto": max(10, actual - 5), "eventos": len(empujes),
                        "texto": f"Empujaste {len(empujes)} veces con más de 40 cm libres: la autonomía es demasiado cautelosa."})
        giros = [e for e in ev if e["tipo"] == "giro_contrario"
                 and min((x for x in ((e.get("dist_cm") or {}).get("izquierda"), (e.get("dist_cm") or {}).get("derecha"))
                          if x is not None), default=999) < 50]
        if len(giros) >= MIN_EVENTOS:
            actual = int(slam.get("cono_lateral_deg", 60))
            sug.append({"id": "cono_lateral_mas", "seccion": "slam", "clave": "cono_lateral_deg", "actual": actual,
                        "propuesto": min(90, actual + 5), "eventos": len(giros),
                        "texto": f"Corregiste {len(giros)} giros con una pared a menos de 50 cm: ampliar el cono lateral."})
        return sug

    def segundos_sin_orden(self) -> float:
        return self.reloj() - max(self.ultima_orden, self.desde)

    def estado(self) -> dict:
        self._caducar(self.reloj())
        return {"activa": self.activa, "modo_previo": self.modo_previo, "desde": self.desde,
                "auto_cmd": {"v": self.auto_cmd[0], "w": self.auto_cmd[1]},
                "eventos_sesion": self.eventos_sesion, "recientes": self.recientes[-6:]}
