"""Exporta los mapas de `mapas_interiores.py` como mundos del simulador visual (`mundos/interior_<nombre>.json`).

El dashboard los lista en «Cargar mundo»; el visor 3D los pinta como interior (suelo de baldosa, muros enyesados, mesas y estanterías
como bloques macizos, luminarias de techo). Las paredes siguen siendo segmentos: lo que ve el LIDAR del simulador es lo mismo que ve el
entrenamiento. Uso: python3 exportar_mundos_interiores.py"""
import json
from pathlib import Path

import mapas_interiores as MI

DESTINO = Path(__file__).with_name("mundos")


def exportar():
    DESTINO.mkdir(exist_ok=True)
    nombres = []
    for nombre, e in MI.mapas().items():
        xs = [v for s in e.paredes for v in (s[0], s[2])]
        ys = [v for s in e.paredes for v in (s[1], s[3])]
        centro = [list(p) for p in (e.linea_central or [(0.0, 0.0), tuple(e.meta_xy or (e.goal_x, 0.0))])]
        mundo = {"nombre": f"interior_{nombre}", "paredes": [[round(v, 3) for v in s] for s in e.paredes],
                 "sendero": [[round(x, 3), round(y, 3)] for x, y in centro], "ancho_sendero": 0.8, "ancho_paja": 0.0,
                 "plantas": [[round(x, 3), round(y, 3), r, 0, 0.45] for x, y, r in e.macetas if r > 0.04],
                 "escaneos": [], "seco": 0.9, "borde": "paja", "superficie": "baldosa",
                 "objetos": [{"tipo": "pata", "x": round(x, 3), "y": round(y, 3), "radio": r, "altura": 0.7 if r > 0.02 else 0.45, "solido": True}
                             for x, y, r in e.macetas if r <= 0.04],
                 "limites": [round(min(xs) - 0.6, 2), round(min(ys) - 0.6, 2), round(max(xs) + 0.6, 2), round(max(ys) + 0.6, 2)],
                 "descripcion": e.descripcion, "meta": list(e.meta_xy or (e.goal_x, 0.0)), "entrenar": False}
        (DESTINO / f"interior_{nombre}.json").write_text(json.dumps(mundo, ensure_ascii=False, indent=1))
        nombres.append(f"interior_{nombre}")
    print("Exportados:", ", ".join(nombres))


if __name__ == "__main__":
    exportar()
