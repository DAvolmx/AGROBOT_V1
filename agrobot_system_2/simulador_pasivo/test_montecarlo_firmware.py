import unittest
import fallos_3d as F
import montecarlo_firmware as MC


class MonteCarloTests(unittest.TestCase):
    def setUp(self):
        self.est = F.Estabilidad()
        self.sits = MC.generar_situaciones(400, semilla=1)

    def _cfg(self, **kw):
        base = {"max_v_mps": 1.0, "max_omega_rps": 3.0, "escala_velocidad": 0.5,
                "max_linear_accel_mps2": 2.0, "inclinacion_precaucion_deg": 22.0}
        base.update(kw)
        return base

    def test_mas_lento_vuelca_menos(self):
        rapido = MC.evaluar(self._cfg(max_v_mps=1.5, escala_velocidad=1.0), self.sits, self.est)
        lento = MC.evaluar(self._cfg(max_v_mps=0.4, escala_velocidad=0.4), self.sits, self.est)
        self.assertLess(lento["vuelco_rate"], rapido["vuelco_rate"])

    def test_la_busqueda_mejora_sobre_la_config_actual(self):
        cands, _ = MC.buscar(n=300, sits_n=300, semilla=2)
        mejor = next(c for c in cands if c[0] != "actual")
        actual = next(c for c in cands if c[0] == "actual")
        self.assertLess(mejor[2]["coste"], actual[2]["coste"])
        self.assertLess(mejor[2]["vuelco_rate"], actual[2]["vuelco_rate"])

    def test_existe_un_suelo_de_vuelco_por_geometria(self):
        # Ni a paso de tortuga se elimina: los bultos >=8 cm vuelcan por
        # geometría a cualquier velocidad. Es un problema de DISEÑO, no de
        # firmware. (El 5% de la distribución son bultos de 8-12 cm.)
        casi_quieto = MC.evaluar(self._cfg(max_v_mps=0.2, escala_velocidad=0.3),
                                 self.sits, self.est)
        self.assertGreater(casi_quieto["vuelco_rate"], 0.0)


if __name__ == "__main__":
    unittest.main()
