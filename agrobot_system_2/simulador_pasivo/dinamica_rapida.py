"""Núcleo COMPILADO (numba) del paso de dinámica de dinamica_pasiva.DinamicaPasiva.step (2026-10-22).

El 82 % del tiempo de un episodio simulado se iba en ese paso (miles de arrays NumPy diminutos por substep). Este módulo lo reescribe con
escalares y un solucionador 4x4 propio, compilado con numba: misma matemática línea a línea (verificada contra la versión de Python en
test_dinamica_rapida.py). Si numba no está instalado o AGROBOT_NUMBA=off, DinamicaPasiva sigue usando la versión de siempre.
"""
import math
import os

import numpy as np

try:
    if os.environ.get("AGROBOT_NUMBA", "").lower() == "off":
        raise ImportError
    from numba import njit
    DISPONIBLE = True
except ImportError:
    DISPONIBLE = False

    def njit(*a, **k):
        def deco(f):
            return f
        return deco(a[0]) if a and callable(a[0]) else deco


@njit(cache=True)
def _resolver(M, b):
    """Gauss con pivoteo parcial para el sistema 4x4 M x = b."""
    A = M.copy()
    x = b.copy()
    n = 4
    for i in range(n):
        p = i
        mx = abs(A[i, i])
        for r in range(i + 1, n):
            if abs(A[r, i]) > mx:
                mx = abs(A[r, i])
                p = r
        if p != i:
            for c in range(n):
                t = A[i, c]; A[i, c] = A[p, c]; A[p, c] = t
            t = x[i]; x[i] = x[p]; x[p] = t
        piv = A[i, i]
        for r in range(i + 1, n):
            f = A[r, i] / piv
            if f != 0.0:
                for c in range(i, n):
                    A[r, c] -= f * A[i, c]
                x[r] -= f * x[i]
    for i in range(n - 1, -1, -1):
        s = x[i]
        for c in range(i + 1, n):
            s -= A[i, c] * x[c]
        x[i] = s / A[i, i]
    return x


@njit(cache=True)
def paso_rapido(q, dq, wheels, P, toe, gain, wt, usa_wt, libres, usa_libres, linear, angular, count, hdt, stats):
    """Avanza `count` substeps de `hdt`. Modifica q, dq, wheels y stats = [slip, max_force_ratio, stop_impacts, stop_energy_loss] EN SITIO.
    P = [wheelbase, front_outer, rear_outer, tire_width, radius, trail, chassis_mass, front_mass, chassis_inertia, front_inertia, cg_x,
         wheel_inertia, mu, longitudinal, lateral, pivot_damping, pivot_friction, motor_gain, max_torque, front_drive_bias, rolling_drag,
         limit, drive(0 all,1 front,2 rear)]"""
    wb, f_out, r_out, tw, rad, trail = P[0], P[1], P[2], P[3], P[4], P[5]
    mc, mf, Ic, If, cgx = P[6], P[7], P[8], P[9], P[10]
    wI, mu, lon, lat_k, pdamp, pfric = P[11], P[12], P[13], P[14], P[15], P[16]
    mg, mtq, fbias, rdrag, lim, drive = P[17], P[18], P[19], P[20], P[21], int(P[22])
    nf = 9.80665 * (mc * cgx / (wb + trail) + mf) / 2.0
    nr = ((mc + mf) * 9.80665 - 2.0 * nf) / 2.0
    M = np.zeros((4, 4))
    Q = np.zeros(4)
    wdot = np.zeros(4)
    a = np.zeros(4)
    for _ in range(count):
        psi, delta = q[2], q[3]
        c, s = math.cos(psi), math.sin(psi)
        # ---- mass_bias
        for i in range(4):
            for j in range(4):
                M[i, j] = 0.0
        k1, k2 = cgx, wb + trail
        for (m_, k) in ((mc, k1), (mf, k2)):
            M[0, 0] += m_
            M[1, 1] += m_
            M[0, 2] += -m_ * k * s
            M[2, 0] += -m_ * k * s
            M[1, 2] += m_ * k * c
            M[2, 1] += m_ * k * c
            M[2, 2] += m_ * k * k
        M[2, 2] += Ic + If
        M[2, 3] += If
        M[3, 2] += If
        M[3, 3] += If
        r2 = dq[2] * dq[2]
        h0 = mc * (-cgx * r2 * c) + mf * (-(wb + trail) * r2 * c)
        h1 = mc * (-cgx * r2 * s) + mf * (-(wb + trail) * r2 * s)
        h2 = mc * (-s * (-cgx * r2 * c) * cgx + c * (-cgx * r2 * s) * cgx) + mf * (-s * (-(wb + trail) * r2 * c) * (wb + trail) + c * (-(wb + trail) * r2 * s) * (wb + trail))
        for i in range(4):
            Q[i] = 0.0
            wdot[i] = 0.0
        slip_sum = 0.0
        idx = 0
        for fr in range(2):
            front = fr == 1
            alpha = psi + (delta if front else 0.0)
            tax0, tax1 = math.cos(alpha), math.sin(alpha)
            nax0, nax1 = -tax1, tax0
            track = (f_out if front else r_out) - tw
            for sg in range(2):
                sign = 1.0 if sg == 0 else -1.0
                if front:
                    o0 = -trail * tax0 + sign * track / 2.0 * nax0
                    o1 = -trail * tax1 + sign * track / 2.0 * nax1
                else:
                    o0 = sign * track / 2.0 * nax0
                    o1 = sign * track / 2.0 * nax1
                tn0, tn1 = -o1, o0
                J02, J12 = tn0, tn1
                J03, J13 = 0.0, 0.0
                if front:
                    J02 += wb * (-s)
                    J12 += wb * c
                    J03, J13 = tn0, tn1
                aw = alpha + toe[idx]
                t0, t1 = math.cos(aw), math.sin(aw)
                n0, n1 = -t1, t0
                v0 = dq[0] + J02 * dq[2] + J03 * dq[3]
                v1 = dq[1] + J12 * dq[2] + J13 * dq[3]
                slip = rad * wheels[idx] - (t0 * v0 + t1 * v1)
                lat = n0 * v0 + n1 * v1
                f0, f1 = lon * slip, -lat_k * lat
                cap = mu * max(0.0, nf if idx >= 2 else nr)
                fn = math.sqrt(f0 * f0 + f1 * f1)
                esc = min(1.0, cap / max(fn, 1e-12))
                f0 *= esc
                f1 *= esc
                g0 = f0 * t0 + f1 * n0
                g1 = f0 * t1 + f1 * n1
                Q[0] += g0
                Q[1] += g1
                Q[2] += J02 * g0 + J12 * g1
                Q[3] += J03 * g0 + J13 * g1
                if usa_wt:
                    target = wt[idx] / rad
                else:
                    target = (linear + (-1.0 if idx % 2 == 0 else 1.0) * angular * track / 2.0) / rad
                powered = (drive == 0) or ((drive == 1) == front)
                bias = (fbias if front else 1.0) * gain[idx]
                motor = 0.0
                if powered and not (usa_libres and libres[idx]):
                    motor = mg * bias * (target - wheels[idx])
                    lo, hi = -mtq * bias, mtq * bias
                    motor = lo if motor < lo else (hi if motor > hi else motor)
                wdot[idx] = (motor - rad * f0 - rdrag * wheels[idx]) / wI
                slip_sum += math.sqrt(slip * slip + lat * lat)
                fr_ = math.sqrt(f0 * f0 + f1 * f1) / max(cap, 1e-12)
                if fr_ > stats[1]:
                    stats[1] = fr_
                idx += 1
        Q[3] += -pdamp * dq[3] - pfric * math.tanh(dq[3] / 0.01)
        rhs = np.empty(4)
        rhs[0] = Q[0] - h0
        rhs[1] = Q[1] - h1
        rhs[2] = Q[2] - h2
        rhs[3] = Q[3]
        x = _resolver(M, rhs)
        for i in range(4):
            dq[i] += x[i] * hdt
            wheels[i] += wdot[i] * hdt
            q[i] += dq[i] * hdt
        # ---- enforce_stop (con la M del inicio del substep)
        if abs(q[3]) >= lim:
            side = 1.0 if q[3] >= 0.0 else -1.0
            q[3] = side * lim
            if side * dq[3] > 0.0:
                for i in range(4):
                    a[i] = 0.0
                a[3] = side
                z = _resolver(M, a)
                speed = a[3] * dq[3]
                az = a[3] * z[3]
                stats[3] += 0.5 * speed * speed / az
                for i in range(4):
                    dq[i] -= z[i] * speed / az
                stats[2] += 1.0
        q[2] = math.atan2(math.sin(q[2]), math.cos(q[2]))
        stats[0] = slip_sum / 4.0
