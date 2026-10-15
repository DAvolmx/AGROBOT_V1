import math
import unittest

from controlador_zigzag import ControladorZigzag


class ZigzagTests(unittest.TestCase):
    def setUp(self):
        self.c = ControladorZigzag(v=0.5)

    def test_en_recto_el_zigzag_es_de_media_cero(self):
        # Sin error de rumbo: avanza y el omega alterna signo, media ~0.
        omegas = [self.c.ordenar(0.0, t * 0.05)[1] for t in range(200)]
        vs = [self.c.ordenar(0.0, t * 0.05)[0] for t in range(200)]
        self.assertTrue(all(v == self.c.v for v in vs))        # siempre avanza
        self.assertLess(abs(sum(omegas) / len(omegas)), 0.1)    # media ~0
        self.assertTrue(any(o > 0 for o in omegas) and any(o < 0 for o in omegas))

    def test_en_curva_alterna_giro_en_sitio_y_avance(self):
        err = math.radians(40)                                 # error grande: modo giro
        muestras = [self.c.ordenar(err, t * 0.05) for t in range(80)]
        hay_giro = any(abs(o) > 0 and v < self.c.v for v, o in muestras)   # giro (con avance leve)
        hay_avance = any(v == self.c.v and o == 0.0 for v, o in muestras)  # avance recto
        self.assertTrue(hay_giro, "debe haber pulsos de giro")
        self.assertTrue(hay_avance, "debe haber pulsos de avance")

    def test_gira_hacia_el_error(self):
        _, oizq = self.c.ordenar(math.radians(40), 0.0)        # error +: gira +
        _, oder = self.c.ordenar(-math.radians(40), 0.0)
        self.assertGreater(oizq, 0)
        self.assertLess(oder, 0)

    def test_modo_segun_umbral(self):
        self.assertEqual(self.c.modo(math.radians(5)), "recto")
        self.assertEqual(self.c.modo(math.radians(30)), "giro")


if __name__ == "__main__":
    unittest.main()
