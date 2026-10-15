#!/usr/bin/env python3
"""Percepción CÁMARA + SERVO TILT para ver lo bajo y frenar antes de volcar.

El LIDAR está ALTO: casi no ve los montículos de 5-8 cm que vuelcan/atrancan al
robot. La cámara, con el servo de tilt mirando ACTIVAMENTE el suelo cercano, sí
los ve. Si detecta un montículo peligroso, frena a la velocidad segura (o para)
ANTES de chocar: es la defensa proactiva que faltaba.

Este módulo modela esa cadena y mide cuánto baja los vuelcos frente a:
  · lidar_alto   : solo el LIDAR alto (lo de hoy): no ve lo bajo.
  · camara_fija  : cámara mirando al frente, sin barrido: ve regular lo cercano.
  · camara_tilt  : cámara con barrido activo de tilt (lejos sendero / cerca suelo):
                   ve bien lo bajo. <- la propuesta.

Frenar es barato: a 0.75 m/s, frenar a la velocidad segura cuesta <7 cm. Por eso
el límite NO es el freno sino la DETECCIÓN. Las curvas prob_deteccion(h) son
hipótesis a CALIBRAR con el video real del terreno (reemplazan a estas).

    python3 percepcion_tilt.py           # tabla comparativa
    python3 percepcion_tilt.py --json
"""
import argparse
import json
import math

import fallos_3d as F
from montecarlo_firmware import generar_situaciones

ACCEL_MAX = 3.0
RANGO_DETECCION_M = {"lidar_alto": 4.0, "camara_fija": 1.2, "camara_tilt": 0.9}


def _sig(h, h50, k):
    return 1.0 / (1.0 + math.exp(-k * (h - h50)))


def prob_deteccion(h, modo):
    """Probabilidad de detectar un montículo de altura h (m) según el sensor.
    HIPÓTESIS a calibrar con el video: el LIDAR alto no ve lo bajo; la cámara
    fija lo ve regular; la cámara con tilt activo mirando el suelo cercano lo ve
    bien incluso a pocos cm."""
    if modo == "lidar_alto":
        return _sig(h, 0.18, 50)      # solo ve obstáculos altos (>~15 cm)
    if modo == "camara_fija":
        return _sig(h, 0.09, 40)      # 50% a 9 cm: los de 5 cm se le escapan
    if modo == "camara_tilt":
        return _sig(h, 0.025, 90)     # 50% a 2.5 cm, ~95% a 5 cm
    raise ValueError(modo)


def frena_a_tiempo(v, v_segura, distancia, accel=ACCEL_MAX):
    """¿Puede bajar de v a v_segura en la distancia disponible?"""
    if v <= v_segura:
        return True
    return (v * v - v_segura * v_segura) / (2 * accel) <= distancia


_CACHE_VSEG = {}


def v_segura_para(h, est):
    """Velocidad a la que ese montículo ya NO vuelca (0 si vuelca hasta parado).
    Cacheado por altura en mm (analítico, rápido)."""
    clave = round(h * 1000)
    if clave not in _CACHE_VSEG:
        vc = F.v_critica_obstaculo_rapida(h, est=est, v_max=1.6)
        _CACHE_VSEG[clave] = 1.6 if vc is None else max(0.0, vc - 0.05)
    return _CACHE_VSEG[clave]


def tasa_vuelco(sits, v_obj, modo, est, accel=ACCEL_MAX):
    """Tasa de vuelco esperada con este sensor + frenado proactivo.

    Si detecta el montículo (prob_deteccion) y puede frenar a tiempo a su
    velocidad segura (o parar), no vuelca. Si lo detecta pero ni parando se
    salva (montículo que vuelca por geometría), para y lo esquiva -> no vuelca.
    Si NO lo detecta, vuelca según la física si iba demasiado rápido."""
    rango = RANGO_DETECCION_M[modo]
    total = 0.0
    for h, es_giro, omega_dem, pendiente in sits:
        vuelca_fisica = F.vuelca_obstaculo_energia(h, v_obj, est=est)
        if not vuelca_fisica:
            continue                      # no volcaría igualmente
        p_det = prob_deteccion(h, modo)
        vseg = v_segura_para(h, est)
        # Detectado: frena a vseg (o para si vseg=0) dentro del rango -> salvado.
        salvable = frena_a_tiempo(v_obj, vseg, rango, accel)
        p_salva = p_det if salvable else 0.0
        total += (1.0 - p_salva)          # solo vuelca si NO lo detecta
    return total / len(sits)


def comparar(v_obj=0.75, sits_n=800, semilla=0, est=None):
    est = est or F.Estabilidad()
    sits = generar_situaciones(sits_n, semilla)
    return {m: round(tasa_vuelco(sits, v_obj, m, est) * 100, 1)
            for m in ("lidar_alto", "camara_fija", "camara_tilt")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=0.75)
    ap.add_argument("--situaciones", type=int, default=800)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    est = F.Estabilidad()
    res = {v: comparar(v, a.situaciones, est=est) for v in (0.5, 0.75, 1.0)}
    if a.json:
        print(json.dumps({"estabilidad": est.resumen(), "vuelco_por_sensor_%": res},
                         indent=2, ensure_ascii=False))
        return
    print("Vuelco (%) según el sensor de manejo y la velocidad "
          "(chasis actual 12 kg / CG 20 cm):\n")
    print(f"{'v (m/s)':>8} {'LIDAR alto':>12} {'cámara fija':>13} {'cámara+tilt':>13}")
    for v, r in res.items():
        print(f"{v:>8} {r['lidar_alto']:>11}% {r['camara_fija']:>12}% "
              f"{r['camara_tilt']:>12}%")
    print("\nEl tilt activo ve lo bajo que el LIDAR no ve y frena antes: "
          "es la mayor reducción de vuelcos sin tocar el chasis.")
    print("(curvas de detección = hipótesis; el video del terreno las calibra)")


if __name__ == "__main__":
    main()
