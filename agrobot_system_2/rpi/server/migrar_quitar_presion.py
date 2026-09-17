#!/usr/bin/env python3
"""
Migración de una sola vez: elimina las columnas `pressure` y `temp_bmp` de la
tabla `packets` del histórico, después de que el BMP280 saliera del robot.

Además limpia las menciones al sensor que quedaron DENTRO de datos ya
escritos: las listas `out_of_range` / `sensors_missing` de cada fila y el
último estado en `estado_actual`. Sin esto, el histórico sigue mostrando
"⚠️ fuera de rango: pressure" en cada paquete viejo, avisando de un sensor
que ya no existe. Eso es reescribir un aviso, no borrar una medición: los
valores de presión se van con las columnas, arriba.

Es un script suelto y no una migración automática al arrancar el gateway a
propósito. `DROP COLUMN` destruye datos y no se deshace, y el gateway se
arranca también sobre copias de la base —backups, la que alguien traiga en un
pendrive— donde ese histórico puede ser lo único que quede de una campaña de
campo. Borrar tiene que ser una decisión de quien ejecuta, no un efecto
secundario de encender el servicio.

Uso:
    python3 migrar_quitar_presion.py                 # muestra qué haría
    python3 migrar_quitar_presion.py --ejecutar      # lo hace

Antes de tocar nada copia la base a  <nombre>.pre_presion_<fecha>.db.
"""
import argparse
import json
import shutil
import sqlite3
import sys
import time
from pathlib import Path

COLUMNAS = ("pressure", "temp_bmp")
DB_POR_DEFECTO = Path(__file__).with_name("agrobot_history.db")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--db", type=Path, default=DB_POR_DEFECTO)
    ap.add_argument("--ejecutar", action="store_true",
                    help="sin esto sólo se informa, no se modifica la base")
    args = ap.parse_args()

    if not args.db.exists():
        print(f"✗ no existe {args.db}")
        return 1

    con = sqlite3.connect(args.db)
    columnas = {fila[1] for fila in con.execute("PRAGMA table_info(packets)")}
    sobran = [c for c in COLUMNAS if c in columnas]
    filas = con.execute("SELECT COUNT(*) FROM packets").fetchone()[0]
    avisos = con.execute(
        "SELECT COUNT(*) FROM packets WHERE out_of_range LIKE '%pressure%'"
        " OR out_of_range LIKE '%tempBMP%'"
        " OR sensors_missing LIKE '%pressure%'"
        " OR sensors_missing LIKE '%tempBMP%'").fetchone()[0]

    if not sobran and not avisos:
        print(f"✓ {args.db.name}: ya no queda nada del sensor de presión")
        con.close()
        return 0

    print(f"base:     {args.db}")
    print(f"filas:    {filas} en packets")
    print(f"a quitar: {', '.join(sobran) or 'ninguna columna, ya estaban'}")
    print(f"avisos a limpiar: {avisos} filas nombran el sensor en out_of_range/sensors_missing")

    if not args.ejecutar:
        print("\n(simulación — vuelve a lanzarlo con --ejecutar para aplicarlo)")
        con.close()
        return 0

    # DROP COLUMN necesita SQLite 3.35 o superior. Se comprueba antes de haber
    # tocado nada: fallar a mitad de las dos columnas dejaría la tabla en un
    # estado que no es ni el viejo ni el nuevo.
    if sqlite3.sqlite_version_info < (3, 35, 0):
        print(f"✗ SQLite {sqlite3.sqlite_version} no soporta DROP COLUMN (hace falta 3.35+)")
        con.close()
        return 1

    respaldo = args.db.with_suffix(f".pre_presion_{time.strftime('%Y%m%d_%H%M%S')}.db")
    con.close()
    shutil.copy2(args.db, respaldo)
    print(f"\n✓ respaldo en {respaldo}")

    con = sqlite3.connect(args.db)
    for columna in sobran:
        con.execute(f"ALTER TABLE packets DROP COLUMN {columna}")
        print(f"✓ columna {columna} eliminada")

    # Las dos listas son JSON, así que se reescriben campo a campo en vez de
    # con un REPLACE de texto: un REPLACE dejaría comas sueltas y el
    # dashboard recibiría un JSON roto.
    limpiadas = 0
    for pid, oor, miss in con.execute(
            "SELECT id, out_of_range, sensors_missing FROM packets").fetchall():
        nuevos = []
        for bruto in (oor, miss):
            lista = json.loads(bruto) if bruto else []
            nuevos.append([c for c in lista if c not in ("pressure", "tempBMP")])
        if [json.loads(x) if x else [] for x in (oor, miss)] != nuevos:
            con.execute("UPDATE packets SET out_of_range = ?, sensors_missing = ? WHERE id = ?",
                        (json.dumps(nuevos[0]), json.dumps(nuevos[1]), pid))
            limpiadas += 1
    print(f"✓ avisos del sensor limpiados en {limpiadas} filas")

    # El último estado se guarda como el JSON entero del paquete, así que
    # también arrastra las claves viejas.
    for (bruto,) in con.execute("SELECT packet_json FROM estado_actual WHERE id = 1").fetchall():
        paquete = json.loads(bruto)
        antes = len(paquete)
        for clave in ("pressure", "tempBMP"):
            paquete.pop(clave, None)
        for clave in ("_out_of_range", "_sensors_missing"):
            if isinstance(paquete.get(clave), list):
                paquete[clave] = [c for c in paquete[clave] if c not in ("pressure", "tempBMP")]
        con.execute("UPDATE estado_actual SET packet_json = ? WHERE id = 1", (json.dumps(paquete),))
        print(f"✓ último estado limpiado ({antes} claves antes)")

    con.commit()
    con.execute("VACUUM")          # recupera el espacio de las columnas muertas
    con.close()

    con = sqlite3.connect(args.db)
    quedan = {fila[1] for fila in con.execute("PRAGMA table_info(packets)")}
    despues = con.execute("SELECT COUNT(*) FROM packets").fetchone()[0]
    con.close()
    print(f"✓ filas conservadas: {despues} de {filas}")
    if despues != filas or (set(COLUMNAS) & quedan):
        print("✗ el resultado no es el esperado — restaura desde el respaldo")
        return 1
    print("✓ migración completa")
    return 0


if __name__ == "__main__":
    sys.exit(main())
