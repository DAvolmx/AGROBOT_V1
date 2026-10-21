"""Efecto del croquis del operador (2026-10-20) sobre el simulador: mismas configs y semillas, tres condiciones.

  AGROBOT_GEOMETRIA=anterior                           LIDAR en el centro y ve todo (resultados previos)
  AGROBOT_GEOMETRIA=croquis                            geometría y LIDAR nuevos, macetas altas (0,45 m)
  AGROBOT_GEOMETRIA=croquis AGROBOT_ALTURA_MACETA=0.2  macetas BAJAS: el LIDAR (37,5 cm) pasa por encima
Uso: python3 medir_croquis.py <etiqueta>   (la geometría sale de las variables de entorno)
"""
import json
import sys
from multiprocessing import Pool

import sim_logica_actual as S
import entrenar_interior_estricto as T

if __name__ == "__main__":
    etiqueta = sys.argv[1]
    cfgs = [S.config_actual(), S.config_interior_perfil()]
    with Pool(3) as pool:
        r = T.correr(pool, cfgs, T.TEST)
    salida = {"condicion": etiqueta, "geometria": S.GEOMETRIA, "altura_maceta": S.ALTURA_MACETA_M}
    for n, rs in r.items():
        salida[n] = {**T.resumen(rs), "por_escenario": T.por_esc(rs)}
    print(json.dumps(salida, ensure_ascii=False), flush=True)
