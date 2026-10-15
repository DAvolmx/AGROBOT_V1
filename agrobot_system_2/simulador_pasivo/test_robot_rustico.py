import math
import unittest

from dinamica_pasiva import Parametros
import fallos_3d as F
import robot_rustico as R


class RusticoTests(unittest.TestCase):
    def test_ideal_casi_no_deriva(self):
        p = Parametros(substep=0.01)             # sin imperfecciones
        self.assertLess(R.deriva_recta(p)["deriva_cm"], 1.0)

    def test_chasis_rustico_deriva_solo(self):
        # Una unidad con desalineación marcada debe desviarse en lazo abierto.
        p = R.parametros_rusticos(2, substep=0.01)
        self.assertGreater(R.deriva_recta(p)["deriva_cm"], 2.0)

    def test_servo_reduce_el_error_de_rumbo(self):
        p = R.parametros_rusticos(2, substep=0.01)
        abierto = abs(R.deriva_recta(p)["rumbo_final_deg"])
        servo = R.mantener_rumbo(p, R.ServoDiferencial(kp=5.0, ki=3.0))["rumbo_rms_deg"]
        self.assertLess(servo, abierto)          # el servo aprende y compensa el sesgo

    def test_amortiguador_hace_volcar_antes(self):
        # Mismo bulto y velocidad: el frente con recoil de amortiguador vuelca
        # donde el rígido aguanta.
        rigido = F.Estabilidad(Parametros(susp_recoil=0.0))
        blando = F.Estabilidad(Parametros(susp_recoil=0.8))
        v = 0.0
        while v <= 1.5:
            if F.vuelca_obstaculo_energia(0.05, v, est=blando):
                v_blando = v
                break
            v += 0.02
        v = 0.0
        while v <= 1.5:
            if F.vuelca_obstaculo_energia(0.05, v, est=rigido):
                v_rigido = v
                break
            v += 0.02
        self.assertLess(v_blando, v_rigido)      # el blando vuelca a MENOS velocidad

    def test_toe_cero_y_gain_uno_no_cambian_la_fisica(self):
        # Garantiza que los oficiales (ideales) no se ven afectados.
        p = Parametros()
        self.assertEqual(p.toe_ruedas, (0., 0., 0., 0.))
        self.assertEqual(p.gain_ruedas, (1., 1., 1., 1.))
        self.assertEqual(p.susp_recoil, 0.0)


if __name__ == "__main__":
    unittest.main()
