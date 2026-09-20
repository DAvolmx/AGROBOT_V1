"""
Historial de simulaciones y de modelos entrenados.
==================================================

Para qué: después de varias noches de entrenamiento hay una carpeta con
`politica_tanda007.pt`, `politica_tanda031.pt`, `politica.pt`… y ninguna
forma de saber cuál es mejor. Peor: el último no tiene por qué ser el mejor
—el entrenamiento no mejora monótonamente— así que "usa el más nuevo" es una
mala regla. Esto lleva la cuenta y los ordena por lo que hacen, no por fecha.

Dos registros, en `~/.agrobot/aprendizaje/`:

  · `historial.jsonl` — una línea por suceso (episodio, tanda de
    entrenamiento, búsqueda de Montecarlo). Se añade, nunca se reescribe: un
    fichero de líneas sueltas sobrevive a que se corte la luz a mitad de una
    noche de entrenamiento, y un JSON grande no.
  · `modelos.json` — la ficha de cada `.pt`: cuándo salió, con cuántos
    ejemplos, qué acierto de validación, y **cómo le fue en cada escenario**.

La validación es lo que da el orden, y se hace **en varios escenarios**
a propósito. Un modelo entrenado toda la noche sobre la huerta del usuario
puede quedar perfecto ahí y estrellarse en un pasillo; si solo se mirara su
propio mapa, se elegiría ese. Por eso cada modelo corre el mismo banco de
escenarios con las MISMAS semillas, y se le puntúa el conjunto.

La puntuación es la de `montecarlo.PESOS` (menos es mejor): pisar plantas y
chocar cuestan caro, el desvío en proporción, avanzar suma. Se usa la misma
a propósito, para que "mejor modelo" y "mejor configuración" signifiquen lo
mismo y se puedan comparar cambios de una y de otra.
"""
import json
import os
import time
from datetime import datetime
from pathlib import Path

BASE = Path(os.environ.get("AGROBOT_APRENDIZAJE",
                           Path.home() / ".agrobot" / "aprendizaje"))
HISTORIAL = BASE / "historial.jsonl"
MODELOS = BASE / "modelos.json"

# Banco de validación: la huerta del usuario MÁS escenarios que no se parecen
# a ella. Sin los segundos, un modelo sobreajustado a su mapa saldría primero.
BANCO = ("huerta_real", "bucle", "hileras", "pasillo", "callejon")
SEMILLAS = (7, 23)
SEGUNDOS = 90


def _ahora() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M")


def anotar(tipo: str, datos: dict) -> dict:
    """Añade una línea al historial. Nunca revienta el proceso que lo llama."""
    fila = {"cuando": _ahora(), "t": round(time.time()), "tipo": tipo, **datos}
    try:
        BASE.mkdir(parents=True, exist_ok=True)
        with HISTORIAL.open("a", encoding="utf-8") as f:
            f.write(json.dumps(fila, ensure_ascii=False) + "\n")
    except OSError as exc:
        # Que no se pueda escribir el historial no puede tumbar ocho horas de
        # entrenamiento: se avisa y se sigue.
        print(f"[historial] no se pudo anotar: {exc}")
    return fila


def leer(tipo=None, limite=200) -> list[dict]:
    """Las últimas anotaciones, de la más nueva a la más vieja."""
    if not HISTORIAL.exists():
        return []
    filas = []
    for linea in HISTORIAL.read_text(encoding="utf-8").splitlines():
        linea = linea.strip()
        if not linea:
            continue
        try:
            d = json.loads(linea)
        except json.JSONDecodeError:
            continue        # línea a medias de un corte: se ignora, no se pierde el resto
        if tipo is None or d.get("tipo") == tipo:
            filas.append(d)
    return filas[::-1][:limite]


def _fichas() -> dict:
    if not MODELOS.exists():
        return {}
    try:
        return json.loads(MODELOS.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return {}


def _guardar_fichas(fichas: dict):
    BASE.mkdir(parents=True, exist_ok=True)
    # Escritura atómica: si se corta a mitad, el fichero viejo sigue entero.
    tmp = MODELOS.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(fichas, ensure_ascii=False, indent=1), encoding="utf-8")
    tmp.replace(MODELOS)


def registrar_modelo(ruta, **datos) -> dict:
    """Apunta un .pt recién entrenado (aún sin validar)."""
    ruta = Path(ruta)
    fichas = _fichas()
    ficha = fichas.get(ruta.name, {})
    ficha.update({"archivo": ruta.name, "ruta": str(ruta), "entrenado": _ahora(), **datos})
    fichas[ruta.name] = ficha
    _guardar_fichas(fichas)
    anotar("entrenamiento", {"modelo": ruta.name, **datos})
    return ficha


def validar_modelo(ruta, banco=BANCO, semillas=SEMILLAS, segundos=SEGUNDOS,
                   cfg=None, realismo=1.0, avisar=print) -> dict:
    """Corre el modelo por todo el banco y guarda la nota de cada escenario.

    Devuelve la ficha con `escenarios` (una entrada por mapa) y `puntos`
    (la media; menos es mejor)."""
    import montecarlo

    ruta = Path(ruta)
    cfg = json.loads(json.dumps(cfg)) if cfg else _cfg_por_defecto()
    cfg["autonomia"]["politica_activa"] = True

    por_escenario, todos = {}, []
    for nombre in banco:
        resultados = []
        for semilla in semillas:
            try:
                resultados.append(_correr(nombre, cfg, segundos, semilla, realismo, ruta))
            except Exception as exc:
                avisar(f"[historial] {nombre}/{semilla} falló: {type(exc).__name__}: {exc}")
        if not resultados:
            continue
        por_escenario[nombre] = {
            "puntos": round(montecarlo.puntuar(resultados), 2),
            "en_sendero_pct": round(_media(resultados, "en_sendero_pct"), 1),
            "desvio_cm": round(_media(resultados, "desvio_ruta_medio_cm"), 1),
            "plantas_pisadas": round(_media(resultados, "plantas_pisadas"), 2),
            "colisiones": round(_media(resultados, "colisiones"), 2),
            "recorrido_m": round(_media(resultados, "recorrido_m"), 1),
            "episodios": len(resultados),
        }
        todos += resultados
        avisar(f"  {nombre:>14s}: {por_escenario[nombre]['puntos']:7.1f} puntos, "
               f"{por_escenario[nombre]['en_sendero_pct']:5.1f} % en sendero")

    fichas = _fichas()
    ficha = fichas.get(ruta.name, {"archivo": ruta.name, "ruta": str(ruta)})
    ficha.update({"validado": _ahora(), "escenarios": por_escenario,
                  "puntos": round(montecarlo.puntuar(todos), 2) if todos else None,
                  "realismo": realismo, "segundos": segundos,
                  "semillas": list(semillas)})
    fichas[ruta.name] = ficha
    _guardar_fichas(fichas)
    anotar("validacion", {"modelo": ruta.name, "puntos": ficha["puntos"],
                          "escenarios": {k: v["puntos"] for k, v in por_escenario.items()}})
    return ficha


def _media(rs, clave):
    vals = [float(r.get(clave, 0) or 0) for r in rs]
    return sum(vals) / len(vals) if vals else 0.0


def _cfg_por_defecto() -> dict:
    ruta = Path(__file__).resolve().parents[1] / "jetson" / "backend" / "agrobot_config.json"
    cfg = json.loads(ruta.read_text())
    cfg["autonomia"].update(sendero_activo=True, sendero_paja=True,
                            sendero_igualar_luz=True, compensar_cabeceo=True)
    return cfg


def _correr(nombre, cfg, segundos, semilla, realismo, ruta_politica):
    """Un episodio. Los mapas dibujados se cargan del disco; el resto son
    escenarios de siempre."""
    import correr as correr_mod
    import mundo as mundo_mod

    guardado = Path(__file__).resolve().parent / "mundos" / f"{nombre}.json"
    if guardado.exists():
        m = mundo_mod.Mundo.desde_dict(json.loads(guardado.read_text()))
        pose = (float(m.sendero[0][0]), float(m.sendero[0][1]), 0.0)
        escenario = (m, pose)
    else:
        escenario = nombre
    ep = correr_mod.Episodio(escenario, cfg, segundos, semilla)
    ep.robot.realismo = realismo
    if ruta_politica is not None:
        # El episodio no lee la config para esto: se le enchufa la Politica
        # YA cargada desde el .pt que toca. Así se puede validar un modelo
        # concreto sin tener que moverlo encima de `politica.pt`.
        from politica import Politica
        pol = Politica(ruta_politica)
        if not pol.disponible:
            raise RuntimeError(f"no se pudo cargar {ruta_politica}")
        ep.politica = pol
    return ep.correr()


def ranking(solo_validados=True) -> list[dict]:
    """Los modelos de MEJOR a PEOR, por la puntuación del banco entero."""
    fichas = [f for f in _fichas().values()
              if not solo_validados or f.get("puntos") is not None]
    # Un modelo sin validar no tiene sitio en el orden: va al final, y se
    # marca, en vez de colarse arriba con un cero.
    return sorted(fichas, key=lambda f: (f.get("puntos") is None, f.get("puntos", 1e9)))


def mejor() -> dict | None:
    r = ranking()
    return r[0] if r else None


def tabla(filas=None) -> str:
    """El ranking en texto, para la terminal."""
    filas = filas if filas is not None else ranking()
    if not filas:
        return "todavía no hay modelos validados"
    anchos = max(len(f["archivo"]) for f in filas)
    escenarios = sorted({e for f in filas for e in f.get("escenarios", {})})
    cab = f"{'#':>2s} {'modelo':<{anchos}s} {'puntos':>8s} " + \
          " ".join(f"{e[:9]:>9s}" for e in escenarios)
    out = [cab, "-" * len(cab)]
    for i, f in enumerate(filas, 1):
        p = f.get("puntos")
        linea = f"{i:2d} {f['archivo']:<{anchos}s} {('%8.1f' % p) if p is not None else '   sin val'} "
        linea += " ".join(f"{f.get('escenarios', {}).get(e, {}).get('puntos', float('nan')):9.1f}"
                          for e in escenarios)
        out.append(linea)
    out.append("")
    out.append("menos puntos es mejor. Pesos: pisar planta 12, choque 25, "
               "desvío 0,35/cm, avance -1,2/m.")
    return "\n".join(out)


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Historial de simulaciones y modelos")
    p.add_argument("--validar", metavar="RUTA", nargs="*",
                   help="valida esos .pt (o todos los de la carpeta si va vacío)")
    p.add_argument("--segundos", type=int, default=SEGUNDOS)
    p.add_argument("--banco", nargs="*", default=list(BANCO))
    p.add_argument("--eventos", action="store_true", help="muestra el historial de sucesos")
    a = p.parse_args()

    if a.validar is not None:
        rutas = [Path(r) for r in a.validar] or sorted(BASE.glob("politica*.pt"))
        if not rutas:
            raise SystemExit(f"no hay modelos en {BASE}")
        for r in rutas:
            print(f"\nvalidando {r.name} en {len(a.banco)} escenarios…", flush=True)
            validar_modelo(r, banco=tuple(a.banco), segundos=a.segundos)
    if a.eventos:
        for f in leer(limite=40):
            print(f"{f['cuando']}  {f['tipo']:<14s} "
                  f"{json.dumps({k: v for k, v in f.items() if k not in ('cuando','t','tipo')}, ensure_ascii=False)}")
    else:
        print()
        print(tabla())
