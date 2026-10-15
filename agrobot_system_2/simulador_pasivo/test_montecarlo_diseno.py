import unittest
import montecarlo_diseno as D
import fallos_3d as F


class DisenoTests(unittest.TestCase):
    def setUp(self):
        self.sits = D.generar_situaciones(400, semilla=3)

    def test_bajar_cg_sube_el_angulo_de_vuelco(self):
        alto = F.Estabilidad(D.parametros_de({**D.ACTUAL, "cg_height": 0.20}))
        bajo = F.Estabilidad(D.parametros_de({**D.ACTUAL, "cg_height": 0.11}))
        self.assertGreater(bajo.resumen()["ang_vuelco_lateral_deg"],
                           alto.resumen()["ang_vuelco_lateral_deg"])

    def test_bajar_cg_reduce_vuelcos(self):
        alto = F.Estabilidad(D.parametros_de({**D.ACTUAL, "cg_height": 0.20}))
        bajo = F.Estabilidad(D.parametros_de({**D.ACTUAL, "cg_height": 0.11}))
        self.assertLess(D.tasa_vuelco(bajo, self.sits, 0.75),
                        D.tasa_vuelco(alto, self.sits, 0.75))

    def test_la_busqueda_encuentra_un_diseno_seguro(self):
        cands = D.buscar(n=400, sits_n=300, semilla=4, v_obj=0.75)
        mejor = next(c for c in cands if c[0] != "actual")
        actual = next(c for c in cands if c[0] == "actual")
        self.assertLess(mejor[2]["vuelco_rate"], actual[2]["vuelco_rate"])
        self.assertLess(mejor[2]["vuelco_rate"], 0.05)
        # Debe respetar el frente estrecho: no ensancharlo a lo bestia.
        self.assertLess(mejor[1]["front_outer"], 0.30)


if __name__ == "__main__":
    unittest.main()
