import unittest, math
import numpy as np
from dinamica_pasiva import DinamicaPasiva, Parametros
import huella_pasiva as huella

class PhysicsTests(unittest.TestCase):
    def test_straight_symmetry(self):
        d=DinamicaPasiva()
        for _ in range(100): d.step(.02,.2,0.)
        self.assertGreater(d.q[0],.2)
        np.testing.assert_allclose(d.q[1:],0,atol=1e-9)
        self.assertLessEqual(d.max_force_ratio,1+1e-10)
    def test_default_4x4_drives_all_wheels(self):
        d=DinamicaPasiva()
        d.step(.02,.2,0.)
        self.assertEqual(d.p.drive,'all')
        self.assertTrue(np.all(d.wheels > 0),d.wheels)
    def test_front_wheels_use_one_shared_passive_pivot(self):
        d=DinamicaPasiva()
        d.q[3]=math.radians(27)
        d.enforce_stop(d.mass_bias()[0])
        self.assertAlmostEqual(d.q[3],math.radians(20))
        self.assertAlmostEqual(d.dq[3],0.)
    def test_front_wheels_rotate_about_fixed_chassis_pivot(self):
        d=DinamicaPasiva()
        d.q[:]=[.3,-.2,.4,0.]
        before=[item[0].copy() for item in d.contacts()]
        d.q[3]=.2
        after=[item[0].copy() for item in d.contacts()]
        # The axle pivot itself is fixed at the wheelbase point. Steering
        # moves wheel centers around it, not the pivot around the rear axle.
        pivot_before=d.q[:2]+d.p.wheelbase*np.array([math.cos(d.q[2]),math.sin(d.q[2])])
        for i in (2,3):
            radius=np.linalg.norm(before[i]-pivot_before)
            self.assertAlmostEqual(np.linalg.norm(after[i]-pivot_before),radius,places=10)
        self.assertAlmostEqual(np.linalg.norm(after[2]-after[3]),d.p.front_outer-d.p.tire_width,places=10)
    def test_front_contact_jacobian_matches_finite_difference(self):
        d=DinamicaPasiva();d.q[:]=[.3,-.2,.4,.2]
        contacts=list(d.contacts());eps=1e-6
        for i in (2,3):
            J=contacts[i][1]
            for j in (2,3):
                original=d.q[j]
                d.q[j]=original+eps;plus=list(d.contacts())[i][0]
                d.q[j]=original-eps;minus=list(d.contacts())[i][0]
                d.q[j]=original
                np.testing.assert_allclose((plus-minus)/(2*eps),J[:,j],atol=1e-8)
    def test_mirror(self):
        a,b=DinamicaPasiva(),DinamicaPasiva()
        for _ in range(100):a.step(.02,.2,.5);b.step(.02,.2,-.5)
        np.testing.assert_allclose(a.q,b.q*np.array([1,-1,-1,-1]),atol=1e-8)
        self.assertLessEqual(abs(a.q[3]),math.radians(20)+1e-12)
    def test_stop_dissipation_coupled(self):
        d=DinamicaPasiva();d.q[3]=d.p.limit+.001;d.dq[:]=[.1,0,.2,1.]
        M,_=d.mass_bias();before=.5*d.dq@M@d.dq;old=d.dq.copy()
        d.enforce_stop(M)
        self.assertLessEqual(.5*d.dq@M@d.dq,before)
        self.assertAlmostEqual(d.dq[3],0)
        self.assertNotAlmostEqual(d.dq[2],old[2])
    def test_positive_mass(self):
        for angle in [-.34,0,.34]:
            d=DinamicaPasiva();d.q[3]=angle;M,_=d.mass_bias()
            self.assertGreater(np.linalg.eigvalsh(M).min(),0)
    def test_convergence(self):
        a,b=DinamicaPasiva(),DinamicaPasiva(parametros=Parametros(substep=.001))
        for _ in range(100):a.step(.02,.2,.4);b.step(.02,.2,.4)
        self.assertLess(np.linalg.norm(a.q-b.q),.02)
    def test_geometry(self):
        boxes=list(huella.partes(0,0,0,0));rear=boxes[1:3];front=boxes[3:5]
        self.assertAlmostEqual(max(b[1]+b[4]/2 for b in rear)-min(b[1]-b[4]/2 for b in rear),.37)
        self.assertAlmostEqual(max(b[1]+b[4]/2 for b in front)-min(b[1]-b[4]/2 for b in front),.19)
        self.assertAlmostEqual(front[0][0]-rear[0][0],.37)
if __name__=='__main__':unittest.main()
