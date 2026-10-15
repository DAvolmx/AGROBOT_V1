import unittest

import fallos_3d as F
import percepcion_tilt as P


class PercepcionTests(unittest.TestCase):
    def setUp(self):
        self.est = F.Estabilidad()
        self.sits = P.generar_situaciones(400, semilla=1)

    def test_deteccion_crece_con_la_altura(self):
        for modo in ("lidar_alto", "camara_fija", "camara_tilt"):
            self.assertLess(P.prob_deteccion(0.02, modo), P.prob_deteccion(0.10, modo))

    def test_tilt_ve_lo_bajo_mucho_mejor_que_el_lidar(self):
        # A 5 cm (el montículo que volcó), el tilt detecta; el LIDAR casi no.
        self.assertGreater(P.prob_deteccion(0.05, "camara_tilt"), 0.8)
        self.assertLess(P.prob_deteccion(0.05, "lidar_alto"), 0.1)

    def test_tilt_reduce_vuelcos_mas_que_fija_y_lidar(self):
        r = {m: P.tasa_vuelco(self.sits, 0.75, m, self.est)
             for m in ("lidar_alto", "camara_fija", "camara_tilt")}
        self.assertLess(r["camara_tilt"], r["camara_fija"])
        self.assertLess(r["camara_fija"], r["lidar_alto"])
        self.assertLess(r["camara_tilt"], 0.10)       # baja a <10%

    def test_frenar_es_barato(self):
        # De 0.75 a 0.4 m/s cabe en pocos cm: el límite es ver, no frenar.
        self.assertTrue(P.frena_a_tiempo(0.75, 0.4, distancia=0.2))


if __name__ == "__main__":
    unittest.main()
