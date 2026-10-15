import unittest
import math
from dataclasses import replace

from dinamica_pasiva import Parametros
import fallos_3d as F


class EstabilidadTests(unittest.TestCase):
    def setUp(self):
        self.est = F.Estabilidad(Parametros())

    def test_angulo_vuelco_es_atan_d_sobre_h(self):
        e = self.est
        self.assertAlmostEqual(e.angulo_vuelco(e.d_lateral()),
                               math.atan2(e.d_lateral(), e.h))

    def test_frente_estrecho_vuelca_mas_facil_que_un_eje_ancho(self):
        # La arista lateral (vía delantera estrecha) debe dar menos margen que
        # si el apoyo fuese tan ancho como el eje trasero por los dos lados.
        e = self.est
        ang_real = e.angulo_vuelco(e.d_lateral())
        ang_ancho = math.atan2(e.hr, e.h)      # si la vía fuese la trasera entera
        self.assertLess(ang_real, ang_ancho)

    def test_cg_mas_alto_reduce_el_angulo_de_vuelco(self):
        bajo = F.Estabilidad(replace(Parametros(), cg_height=.12))
        alto = F.Estabilidad(replace(Parametros(), cg_height=.24))
        self.assertGreater(bajo.angulo_vuelco(bajo.d_lateral()),
                           alto.angulo_vuelco(alto.d_lateral()))

    def test_umbral_cuasiestatico_lateral(self):
        # Justo por debajo de SSF*g no vuelca; justo por encima, sí.
        e = self.est
        a_umbral = (e.d_lateral() / e.h) * F.G
        i = F._inercia_balanceo(e.m, e.p.rear_outer, 2 * e.h)
        bajo = F._simular_pendulo(a_umbral * 0.9, e.d_lateral(), i, e.m, e.h)
        alto = F._simular_pendulo(a_umbral * 1.1, e.d_lateral(), i, e.m, e.h)
        self.assertFalse(bajo['vuelca'])
        self.assertTrue(alto['vuelca'])

    def test_barrera_de_energia_obstaculo(self):
        # Con poca velocidad de balanceo vuelve a apoyarse; con mucha, vuelca.
        e = self.est
        i = F._inercia_balanceo(e.m, e.p.rear_outer, 2 * e.h)
        phi0 = math.radians(10)
        lento = F._simular_pendulo(0., e.d_lateral(), i, e.m, e.h,
                                   phi0=phi0, phidot0=0.2)
        rapido = F._simular_pendulo(0., e.d_lateral(), i, e.m, e.h,
                                    phi0=phi0, phidot0=8.0)
        self.assertFalse(lento['vuelca'])
        self.assertTrue(rapido['vuelca'])

    def test_obstaculo_muy_alto_vuelca_sin_velocidad(self):
        # Si la rueda sube un escalón que ya inclina más que phi_tip, vuelca
        # por geometría aunque v=0.
        e = self.est
        h_tip = 2 * e.hf * math.tan(e.angulo_vuelco(e.d_lateral()))
        r = F.vuelco_obstaculo(h_tip * 1.2, v=0.0, est=e)
        self.assertTrue(r['vuelca'])

    def test_frenada_suave_no_cabecea_pero_brutal_si(self):
        e = self.est
        self.assertFalse(F.cabeceo_frenada(1.5, 3.0, est=e)['vuelca'])
        self.assertTrue(F.cabeceo_frenada(1.5, 40.0, est=e)['vuelca'])

    def test_frontera_omega_decrece_con_velocidad(self):
        e = self.est
        o_lento = F.max_omega_sin_vuelco(0.5, e)
        o_rapido = F.max_omega_sin_vuelco(1.5, e)
        self.assertGreaterEqual(o_lento, o_rapido)

    def test_energia_obstaculo_coincide_con_integracion(self):
        e = self.est
        for h in (0.02, 0.05, 0.08):
            for v in (0.0, 0.3, 0.6, 1.0, 1.5):
                self.assertEqual(F.vuelca_obstaculo_energia(h, v, est=e),
                                 F.vuelco_obstaculo(h, v, est=e)['vuelca'],
                                 msg=f"h={h} v={v}")

    def test_giro_rapido_coincide_con_integracion(self):
        e = self.est
        for v in (0.5, 1.0, 1.5):
            for omega in (1.0, 2.0, 3.0, 4.0, 5.0):
                self.assertEqual(F.vuelca_giro_rapido(v, omega, est=e),
                                 F.vuelco_lateral_giro(v, omega, est=e)['vuelca'],
                                 msg=f"v={v} omega={omega}")


if __name__ == '__main__':
    unittest.main()
