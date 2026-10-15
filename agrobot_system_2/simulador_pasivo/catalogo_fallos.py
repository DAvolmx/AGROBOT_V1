"""Catálogo de fallos del AGROBOT con el diseño nuevo (eje delantero estrecho).

Barre los escenarios de fallo fuera del plano y reporta las FRONTERAS: a partir
de qué giro, qué obstáculo o qué frenada el robot vuelca. Usa fallos_3d.py.
Límites del firmware real (agrobot_config.json): v_max 1.5 m/s, omega_max 4.0
rad/s, accel 3 m/s^2.

    python3 catalogo_fallos.py            # tabla por pantalla
    python3 catalogo_fallos.py --json     # JSON para el dashboard/informe
"""
import argparse
import json
import math

from dinamica_pasiva import Parametros
import fallos_3d as F

V_MAX = 1.5
OMEGA_MAX = 4.0
ACCEL_MAX = 3.0


def catalogo(p=None):
    est = F.Estabilidad(p)
    res = {'estabilidad': est.resumen(), 'limites_firmware':
           {'v_max_mps': V_MAX, 'omega_max_rps': OMEGA_MAX, 'accel_max_mps2': ACCEL_MAX}}

    # 1) Vuelco lateral en giro: omega máximo seguro y radio mínimo por velocidad
    giros = []
    for v in (0.3, 0.5, 0.8, 1.0, 1.2, 1.5):
        o_seg = F.max_omega_sin_vuelco(v, est, omega_max=OMEGA_MAX + 2)
        r_min = (v / o_seg) if o_seg > 1e-6 else float('inf')
        giros.append({'v': v, 'omega_max_seguro_rps': round(o_seg, 2),
                      'radio_min_m': round(r_min, 2) if math.isfinite(r_min) else None,
                      'supera_firmware': o_seg >= OMEGA_MAX})
    res['vuelco_giro'] = giros

    # 2) Vuelco por obstáculo: velocidad crítica por altura de bulto
    obst = []
    for h_cm in (2, 3, 5, 8, 10):
        vc = F.v_critica_obstaculo(h_cm / 100, est, v_max=V_MAX + 0.5)
        r5 = F.vuelco_obstaculo(h_cm / 100, V_MAX, est=est)
        obst.append({'h_obs_cm': h_cm, 'v_critica_mps': vc,
                     'phi0_deg': r5['phi0_deg'],
                     'vuelca_a_vmax': r5['vuelca']})
    res['vuelco_obstaculo'] = obst

    # 3) Cabeceo por frenada brusca
    frenadas = []
    for decel in (ACCEL_MAX, 6.0, 10.0, 20.0):
        r = F.cabeceo_frenada(V_MAX, decel, est=est)
        frenadas.append({'decel_mps2': decel, 'vuelca_morro': r['vuelca'],
                         'rueda_trasera_despega': r['rueda_trasera_despega'],
                         't_parada_s': r['t_parada_s']})
    res['cabeceo_frenada'] = frenadas

    # 4) Peor caso del accidente: obstáculo de 5 cm a velocidad de crucero
    res['caso_accidente'] = F.vuelco_obstaculo(0.05, V_MAX, est=est)
    return res


def imprimir(res):
    e = res['estabilidad']
    print("=" * 66)
    print("  ESTABILIDAD ESTÁTICA DEL DISEÑO NUEVO")
    print("=" * 66)
    print(f"  masa {e['masa_kg']} kg | CG a {e['cg_altura_cm']} cm | "
          f"vía delantera {e['semivia_delantera_cm']*2:.1f} cm / "
          f"trasera {e['semivia_trasera_cm']*2:.1f} cm")
    print(f"  ángulo de vuelco LATERAL : {e['ang_vuelco_lateral_deg']}°  "
          f"(SSF {e['SSF_lateral']}  ->  vuelca con a_y > {e['a_lateral_vuelco_mps2']} m/s²)")
    print(f"  ángulo de vuelco ADELANTE: {e['ang_vuelco_adelante_deg']}°   "
          f"ATRÁS: {e['ang_vuelco_atras_deg']}°")

    print("\n" + "=" * 66)
    print("  1) VUELCO LATERAL EN GIRO  (a_y = v·omega)")
    print("=" * 66)
    print(f"  {'v (m/s)':>8} {'omega seguro':>14} {'radio mín':>11}   nota")
    for g in res['vuelco_giro']:
        nota = "⚠ el firmware permite girar MÁS -> puede volcar" if not g['supera_firmware'] else "ok dentro de límites"
        rr = f"{g['radio_min_m']} m" if g['radio_min_m'] else "—"
        print(f"  {g['v']:>8} {g['omega_max_seguro_rps']:>11} rps {rr:>11}   {nota}")

    print("\n" + "=" * 66)
    print("  2) VUELCO POR OBSTÁCULO  (una rueda delantera sube un bulto)")
    print("=" * 66)
    print(f"  {'bulto':>6} {'v crítica':>11} {'inclina':>9}   nota")
    for o in res['vuelco_obstaculo']:
        vc = f"{o['v_critica_mps']} m/s" if o['v_critica_mps'] is not None else "no vuelca"
        nota = "⚠ vuelca a velocidad de crucero" if o['vuelca_a_vmax'] else "seguro hasta v_max"
        print(f"  {o['h_obs_cm']:>4} cm {vc:>11} {o['phi0_deg']:>7}°   {nota}")

    print("\n" + "=" * 66)
    print("  3) CABECEO POR FRENADA BRUSCA  (desde v_max)")
    print("=" * 66)
    for f in res['cabeceo_frenada']:
        print(f"  decel {f['decel_mps2']:>5} m/s²  ->  vuelco de morro: "
              f"{'SÍ' if f['vuelca_morro'] else 'no':>3}   "
              f"rueda trasera despega: {'sí' if f['rueda_trasera_despega'] else 'no'}")

    print("\n" + "=" * 66)
    print("  CASO DEL ACCIDENTE (2026-10-09): bulto 5 cm a 1.5 m/s")
    print("=" * 66)
    c = res['caso_accidente']
    print(f"  inclinación inicial {c['phi0_deg']}° | ángulo de vuelco "
          f"{c['phi_tip_deg']:.1f}° | VUELCA: {'SÍ' if c['vuelca'] else 'no'}")
    print("=" * 66)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()
    r = catalogo()
    if args.json:
        print(json.dumps(r, indent=2, ensure_ascii=False))
    else:
        imprimir(r)
