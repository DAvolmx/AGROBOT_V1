import unittest
import numpy as np
from fusion_sendero import Calibration,GroundFusion,speed_limit,BLOCKED

class Tests(unittest.TestCase):
    def setUp(self):
        self.c=Calibration(100,100,50,50,.4,0,45,90,1,True)
    def test_centre_ray(self):
        x,y=self.c.ground(50,50,90)
        self.assertAlmostEqual(float(x),.4);self.assertAlmostEqual(float(y),0)
    def test_servo_sign_changes_range(self):
        a,_=self.c.ground(50,50,90);b,_=self.c.ground(50,50,100)
        self.assertLess(b,a)
    def test_no_lidar_return_does_not_clear_ground(self):
        r=GroundFusion().build(np.ones((100,100)),np.ones((100,100)),self.c,90,1,True,True)
        self.assertFalse(r['vision_valid']);self.assertEqual(np.count_nonzero(r['grid']),0)
    def test_moving_servo_rejects_image(self):
        r=GroundFusion().build(np.ones((100,100)),np.ones((100,100)),self.c,90,0,False,True,[(.5,0)])
        self.assertFalse(r['vision_valid']);self.assertEqual(np.count_nonzero(r['grid']==BLOCKED),1)
    def test_unverified(self):
        c=Calibration(100,100,50,50,.4,0,45,90,1)
        with self.assertRaises(ValueError):c.ground(50,50,90)
    def test_braking(self):
        v=speed_limit(.4,.25,.3,maximum_mps=2)
        self.assertLessEqual(v*.25+v*v/(2*.3),.32+1e-12)
        self.assertEqual(speed_limit(None,.25,.3),0)
        self.assertEqual(speed_limit(.4,.25,0),0)
if __name__=='__main__':unittest.main()
