import unittest

import evaluacion as E
from robot_rustico import parametros_rusticos


class EvaluacionTests(unittest.TestCase):
    def test_inmovilidad_puntua_mal(self):
        # Quedarse quieto (progreso 0) debe puntuar PEOR que avanzar bien,
        # aunque el error de sendero sea bajo por no moverse.
        quieto = {"rmse_cm": 1, "progreso": 0.0, "colisiones": 0,
                  "contacto_cultivo": 0, "inestable": False}
        avanza = {"rmse_cm": 5, "progreso": 0.9, "colisiones": 0,
                  "contacto_cultivo": 0, "inestable": False}
        self.assertGreater(E.puntuar(quieto), E.puntuar(avanza))

    def test_penaliza_colision_e_inestabilidad(self):
        base = {"rmse_cm": 2, "progreso": 0.8, "colisiones": 0,
                "contacto_cultivo": 0, "inestable": False}
        self.assertGreater(E.puntuar({**base, "colisiones": 2}), E.puntuar(base))
        self.assertGreater(E.puntuar({**base, "inestable": True}), E.puntuar(base))
        self.assertGreater(E.puntuar({**base, "contacto_cultivo": 1}), E.puntuar(base))

    def test_recta_avanza_y_sigue_el_sendero(self):
        pr = parametros_rusticos(3)
        m = E.simular(E.escenarios()["recta"],
                      E.Controlador(kp=3.5, ki=2.0, v=0.5), pr, 3)
        self.assertGreater(m["progreso"], 0.3)
        self.assertLess(m["rmse_cm"], 15)

    def test_el_control_sigue_mejor_que_sin_control(self):
        # El chasis rústico deriva solo: sin control (kp=ki=0) se sale más que
        # con el PI que aprende y cancela la deriva.
        pr = parametros_rusticos(4)
        esc = E.escenarios()["recta"]
        con = E.simular(esc, E.Controlador(kp=4.0, ki=3.0, v=0.5), pr, 4)
        sin = E.simular(esc, E.Controlador(kp=0.0, ki=0.0, v=0.5, freno=False), pr, 4)
        self.assertLess(con["rmse_cm"], sin["rmse_cm"])


if __name__ == "__main__":
    unittest.main()
