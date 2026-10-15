import math, unittest
from types import SimpleNamespace
import huella

class HuellaTests(unittest.TestCase):
    def test_cabe_recto_en_50cm(self):
        m=SimpleNamespace(paredes=[[-2,-.25,2,-.25],[-2,.25,2,.25]],objetos=[])
        self.assertFalse(huella.choca(m,0,0,0))
        self.assertTrue(huella.choca(m,0,.09,0))
        self.assertTrue(huella.choca(m,0,0,math.pi/4))
    def test_giro_intermedio_no_atraviesa_pared(self):
        m=SimpleNamespace(paredes=[[-2,.26,2,.26]],objetos=[])
        self.assertFalse(huella.choca(m,0,0,0))
        self.assertFalse(huella.choca(m,0,0,math.pi))
        self.assertTrue(huella.barrido(m,0,0,0,0,0,math.pi))
    def test_esquina_circular(self):
        self.assertTrue(huella.circulo(.27,.18,.03,0,0,0))
        self.assertFalse(huella.circulo(.29,.20,.03,0,0,0))
    def test_no_atraviesa_segmento(self):
        m=SimpleNamespace(paredes=[[.5,-1,.5,1]],objetos=[])
        self.assertTrue(huella.barrido(m,0,0,0,1,0,0))

if __name__=='__main__': unittest.main()
