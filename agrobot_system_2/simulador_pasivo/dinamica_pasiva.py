"""Two planar rigid bodies, four slipping wheels and a passive bounded pivot.

SI units. Geometric measurements are confirmed; all dynamic defaults are
explicit hypotheses, NOT identification of the physical robot. No hardware IO.
q origin: rear axle centre. cf=0 is the documented nominal simplification.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
try:
    from .contrato_robot import targets as command_targets
except ImportError:
    from contrato_robot import targets as command_targets


@dataclass
class Parametros:
    wheelbase: float = .37
    front_outer: float = .19
    rear_outer: float = .37
    tire_width: float = .047  # unmeasured
    radius: float = .086     # unmeasured
    trail: float = 0.0       # signed, unmeasured
    # Hipótesis inicial (masa total ~12 kg); pesar el robot para calibrar.
    chassis_mass: float = 10.5
    front_mass: float = 1.5
    chassis_inertia: float = .27
    front_inertia: float = .022
    cg_x: float = .185
    # Hipótesis visual ~0.20 m; medir antes de usar predicciones de vuelco.
    cg_height: float = .20
    wheel_inertia: float = .002
    mu: float = .65
    longitudinal: float = 45.
    lateral: float = 65.
    pivot_damping: float = .04
    pivot_friction: float = .008
    motor_gain: float = .18   # base (rear) proportional gain
    max_torque: float = .8    # base (rear) torque ceiling, N*m
    # Factor relativo hipotético: medir par/velocidad para identificarlo.
    front_drive_bias: float = 2.0
    rolling_drag: float = .001
    limit: float = math.radians(20)
    # El robot usa tracción en las cuatro ruedas; el reparto de par es hipótesis.
    drive: str = 'all'
    substep: float = .002
    # --- Imperfecciones de un chasis RÚSTICO (soldado, algo desalineado) ---
    # Por defecto IDEAL (1.0 / 0.0): los archivos oficiales no cambian. El
    # preset rústico vive en robot_rustico.py. Orden de ruedas: trasera-izq,
    # trasera-der, delantera-izq, delantera-der.
    gain_ruedas: tuple = (1., 1., 1., 1.)   # multiplicador de fuerza por rueda
    toe_ruedas: tuple = (0., 0., 0., 0.)    # desalineación (toe) por rueda, rad
    # Complianza del amortiguador delantero. 0 = rígido (ideal). Al chocar un
    # montículo, la suspensión se comprime y RECULA: devuelve energía al
    # balanceo (peor vuelco) y hace atrancar el frente. Lo usa fallos_3d.
    susp_recoil: float = 0.0


class DinamicaPasiva:
    def __init__(self, pose=(0., 0., 0.), parametros=None):
        self.p = parametros or Parametros()
        if self.p.drive not in ('rear', 'front', 'all'):
            raise ValueError('drive must be rear, front or all')
        if not 0 < self.p.tire_width < self.p.front_outer:
            raise ValueError('invalid tire width')
        self.q = np.array([*pose, 0.], dtype=float)
        self.dq = np.zeros(4)
        self.wheels = np.zeros(4)  # rear L/R, front L/R
        self.stop_impacts = 0
        self.stop_energy_loss = 0.
        self.slip = 0.
        self.max_force_ratio = 0.

    def mass_bias(self):
        p = self.p
        c, s = math.cos(self.q[2]), math.sin(self.q[2])
        def jac(a):
            return np.array([[1., 0., -a*s, 0.], [0., 1., a*c, 0.]])
        jc, jf = jac(p.cg_x), jac(p.wheelbase+p.trail)
        a, b = np.array([0.,0.,1.,0.]), np.array([0.,0.,1.,1.])
        M = p.chassis_mass*jc.T@jc + p.front_mass*jf.T@jf
        M += p.chassis_inertia*np.outer(a,a) + p.front_inertia*np.outer(b,b)
        r2 = self.dq[2]**2
        h = (p.chassis_mass*jc.T@(-p.cg_x*r2*np.array([c,s]))
             + p.front_mass*jf.T@(-(p.wheelbase+p.trail)*r2*np.array([c,s])))
        return M, h

    def contacts(self):
        p = self.p
        psi, delta = self.q[2:]
        c, s = math.cos(psi), math.sin(psi)
        idx = 0
        for front in (False, True):
            alpha = psi + (delta if front else 0.)
            t_ax = np.array([math.cos(alpha), math.sin(alpha)])   # eje del tren
            n_ax = np.array([-t_ax[1], t_ax[0]])
            track = (p.front_outer if front else p.rear_outer)-p.tire_width
            for sign in (1., -1.):
                # El pivote está fijado al chasis a wheelbase metros del eje
                # trasero. El ángulo delta solo rota el conjunto delantero
                # ALREDEDOR de ese pivote; no debe hacer que el propio pivote
                # orbite alrededor del eje trasero.
                offset = -p.trail*t_ax + sign*track/2*n_ax if front else sign*track/2*n_ax
                turn = np.array([-offset[1],offset[0]])
                J = np.array([[1.,0.,turn[0],0.],[0.,1.,turn[1],0.]])
                pos = self.q[:2]+offset
                if front:
                    pivot = self.q[:2] + p.wheelbase*np.array([c,s])
                    pos = pivot + offset
                    # Derivada del pivote respecto al rumbo del chasis y de
                    # cada rueda respecto al ángulo relativo del eje.
                    J[:,2] += p.wheelbase*np.array([-s,c])
                    J[:,3] = turn
                # La POSICIÓN la fija la geometría del tren; la dirección de
                # rodadura de la rueda va girada por su toe (desalineación).
                aw = alpha + p.toe_ruedas[idx]
                t = np.array([math.cos(aw), math.sin(aw)])
                n = np.array([-t[1], t[0]])
                idx += 1
                yield pos, J, t, n

    def enforce_stop(self, M):
        """Inelastic coupled impulse, small-step contact projection.

        Configuration projection is a numerical approximation, checked by
        timestep convergence; velocity impulse conserves generalized momentum
        in unconstrained coordinates and cannot increase kinetic energy.
        """
        if abs(self.q[3]) >= self.p.limit:
            side = math.copysign(1., self.q[3])
            self.q[3] = side*self.p.limit
            if side*self.dq[3] > 0:
                a = np.array([0.,0.,0.,side])
                z = np.linalg.solve(M,a)
                speed = a@self.dq
                self.stop_energy_loss += .5*speed**2/(a@z)
                self.dq -= z*speed/(a@z)
                self.stop_impacts += 1

    def step_firmware(self, dt, linear, angular):
        p=self.p
        v,w,vel=command_targets(linear,angular,p.wheelbase,p.front_outer,p.rear_outer,p.tire_width)
        self.step(dt,v,w,wheel_targets=vel)

    def step(self, dt, linear, angular, wheel_targets=None, libres=None):
        """`libres`: lista de 4 bool (orden de `wheels`). Una rueda LIBRE no recibe par del
        motor: gira por rozamiento. Es lo que hace el firmware con duty 0 (`detachMotor`:
        FI=BI=LOW, TA6586 en alta impedancia). Sin esto, un objetivo de velocidad 0 FRENA la
        rueda activamente, que es lo contrario (añadido el 2026-10-15; por defecto None =
        comportamiento anterior, así que nada previo cambia)."""
        p = self.p
        if dt <= 0: return
        count = max(1,math.ceil(dt/p.substep))
        hdt = dt/count
        for _ in range(count):
            M,h = self.mass_bias()
            Q = np.zeros(4)
            wheel_dot = np.zeros(4)
            # Nominal quasi-static axle loads, cf=0. No roll/load transfer.
            nf = 9.80665*(p.chassis_mass*p.cg_x/(p.wheelbase+p.trail)+p.front_mass)/2
            nr = ((p.chassis_mass+p.front_mass)*9.80665-2*nf)/2
            slips = []
            for i,(_,J,t,n) in enumerate(self.contacts()):
                vel = J@self.dq
                slip = p.radius*self.wheels[i]-t@vel
                lat = n@vel
                force = np.array([p.longitudinal*slip,-p.lateral*lat])
                cap = p.mu*max(0.,nf if i>=2 else nr)
                force *= min(1.,cap/max(np.linalg.norm(force),1e-12))
                Q += J.T@(force[0]*t+force[1]*n)
                track = (p.front_outer if i>=2 else p.rear_outer)-p.tire_width
                target = (wheel_targets[i] if wheel_targets is not None else (linear + (-1 if i%2==0 else 1)*angular*track/2))/p.radius
                front = i>=2
                powered = p.drive=='all' or (p.drive=='front')==front
                # Fuerza por rueda: bias delantero (motores fuertes delante) y
                # el multiplicador rústico por rueda (asimetría de soldadura).
                bias = (p.front_drive_bias if front else 1.) * p.gain_ruedas[i]
                motor = np.clip(p.motor_gain*bias*(target-self.wheels[i]),
                                -p.max_torque*bias, p.max_torque*bias) if (powered and not (libres is not None and libres[i])) else 0.
                wheel_dot[i] = (motor-p.radius*force[0]-p.rolling_drag*self.wheels[i])/p.wheel_inertia
                slips.append(math.hypot(slip,lat))
                self.max_force_ratio = max(self.max_force_ratio,float(np.linalg.norm(force)/max(cap,1e-12)))
            Q[3] += -p.pivot_damping*self.dq[3]-p.pivot_friction*math.tanh(self.dq[3]/.01)
            self.dq += np.linalg.solve(M,Q-h)*hdt
            self.wheels += wheel_dot*hdt
            self.q += self.dq*hdt
            self.enforce_stop(M)
            self.q[2] = math.atan2(math.sin(self.q[2]),math.cos(self.q[2]))
            self.slip = float(np.mean(slips))
        if not np.all(np.isfinite(self.q)):
            raise FloatingPointError('Non-finite dynamics')

    def metadata(self):
        return {'parameters':asdict(self.p),'calibrated':False,
                'drive_assumption':self.p.drive,'model':'passive_two_body_v3'}
