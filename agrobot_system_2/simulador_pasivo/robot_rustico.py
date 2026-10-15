"""Robot RÚSTICO: chasis soldado y algo desalineado, y un control que lo maneja
con precisión de servo usando solo los motores (no hay servo de dirección).

Contenido AISLADO a propósito (el operador pidió no meter lo rústico en los
archivos oficiales): las imperfecciones viven aquí como un preset de
`Parametros`; los valores por defecto del modelo siguen siendo ideales.

Imperfecciones que modela (medidas como hipótesis de un robot hecho a mano):
  · toe: cada rueda apunta un pelín girada (soldadura) -> el robot DERIVA solo.
  · fuerza por rueda asimétrica; las TRASERAS más flojas (el eje principal de
    manejo es el DELANTERO, ya con front_drive_bias).
  · pivote delantero tosco (más fricción).
  · amortiguador delantero que RECULA al chocar (susp_recoil) -> atranca y vuelca.

El "servo de precisión con los motores" es `ServoDiferencial`: un lazo cerrado
PI sobre el rumbo cuyo término integral APRENDE la deriva sistemática del chasis
rústico y la cancela, así los motores toscos mantienen el rumbo como un servo.
"""
import math
import numpy as np

from dinamica_pasiva import DinamicaPasiva, Parametros


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parametros_rusticos(semilla=0, **over):
    """Un 'ejemplar' rústico reproducible (cada semilla = otra unidad soldada)."""
    rng = np.random.default_rng(semilla)
    toe = tuple(float(rng.normal(0.0, math.radians(1.5))) for _ in range(4))
    base = [0.88, 0.88, 1.0, 1.0]        # traseras más flojas; delanteras nominal
    gain = tuple(float(max(0.5, b * (1 + rng.normal(0.0, 0.08)))) for b in base)
    campos = dict(toe_ruedas=toe, gain_ruedas=gain,
                  pivot_friction=0.02, pivot_damping=0.06,   # pivote oxidado/tosco
                  mu=0.6, susp_recoil=0.6)
    campos.update(over)
    return Parametros(**campos)


class ServoDiferencial:
    """Servo de precisión EMULADO con los motores (control diferencial).

    PI sobre el error de rumbo. El integral aprende la deriva constante del
    chasis (toe, motores dispares) y la compensa; con anti-windup y saturación
    de omega para no pedir a los motores más de lo que dan."""

    def __init__(self, v=0.4, kp=3.0, ki=1.5, omega_max=2.0, i_max=2.0):
        self.v, self.kp, self.ki = v, kp, ki
        self.omega_max, self.i_max = omega_max, i_max
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def paso(self, error_rumbo, dt):
        e = _wrap(error_rumbo)
        self.integral = float(np.clip(self.integral + e * dt, -self.i_max, self.i_max))
        omega = float(np.clip(self.kp * e + self.ki * self.integral,
                              -self.omega_max, self.omega_max))
        return self.v, omega


def deriva_recta(p, dist=3.0, v=0.4, dt=0.04):
    """Manda 'recto' en LAZO ABIERTO (omega=0) y mide cuánto se desvía el chasis
    rústico: deriva lateral (cm) y error de rumbo final (grados) por metro."""
    d = DinamicaPasiva(parametros=p)
    rec, x0, y0 = 0.0, 0.0, 0.0
    for _ in range(int(dist / v / dt) + 1):
        d.step(dt, v, 0.0)
        x, y = d.q[:2]
        rec += math.hypot(x - x0, y - y0); x0, y0 = x, y
    return {"deriva_cm": round(abs(d.q[1]) * 100, 1),
            "rumbo_final_deg": round(math.degrees(d.q[2]), 1),
            "recorrido_m": round(rec, 2)}


def mantener_rumbo(p, ctrl, t_max=8.0, dt=0.04, rumbo_obj=0.0):
    """Mantiene un rumbo con el ServoDiferencial sobre el chasis rústico. Mide
    precisión: RMS del error de rumbo (grados) y deriva lateral final (cm)."""
    d = DinamicaPasiva(parametros=p)
    ctrl.reset()
    errs = []
    for _ in range(int(t_max / dt)):
        e = _wrap(rumbo_obj - d.q[2])
        v, omega = ctrl.paso(e, dt)
        d.step(dt, v, omega)
        errs.append(math.degrees(abs(_wrap(rumbo_obj - d.q[2]))))
    # precisión en régimen (último 40 %), ya cancelada la deriva inicial
    reg = errs[int(len(errs) * 0.6):]
    return {"rumbo_rms_deg": round(float(np.sqrt(np.mean(np.square(reg)))), 2),
            "rumbo_rms_total_deg": round(float(np.sqrt(np.mean(np.square(errs)))), 2),
            "deriva_cm": round(abs(d.q[1]) * 100, 1)}


def optimizar_servo(p, semillas=(0,), t_max=8.0):
    """Busca las ganancias (kp, ki) que dan más precisión de rumbo en el chasis
    rústico. Devuelve el mejor ServoDiferencial y su precisión."""
    mejor = None
    for kp in (2.5, 3.5, 5.0):
        for ki in (0.5, 1.5, 3.0):
            rms = []
            for s in semillas:
                pr = p if len(semillas) == 1 else parametros_rusticos(s, substep=0.01)
                c = ServoDiferencial(kp=kp, ki=ki)
                rms.append(mantener_rumbo(pr, c, t_max=t_max)["rumbo_rms_deg"])
            m = float(np.mean(rms))
            if mejor is None or m < mejor[0]:
                mejor = (m, kp, ki)
    _, kp, ki = mejor
    return {"kp": kp, "ki": ki, "rumbo_rms_deg": round(mejor[0], 2),
            "servo": ServoDiferencial(kp=kp, ki=ki)}


if __name__ == "__main__":
    print("ROBOT RÚSTICO — deriva del chasis y precisión del servo diferencial\n", flush=True)
    for s in range(4):
        p = parametros_rusticos(s, substep=0.01)
        abierto = deriva_recta(p)
        srv = ServoDiferencial(kp=3.5, ki=2.0)
        cerrado = mantener_rumbo(p, srv)
        print(f"  unidad #{s}: lazo ABIERTO deriva {abierto['deriva_cm']:>5} cm, "
              f"rumbo {abierto['rumbo_final_deg']:>5}° en {abierto['recorrido_m']} m"
              f"  ->  SERVO: error {cerrado['rumbo_rms_deg']}° rms, "
              f"deriva {cerrado['deriva_cm']} cm", flush=True)
    print("\nGanancias óptimas (media de 4 unidades rústicas):", flush=True)
    opt = optimizar_servo(None, semillas=(0, 1, 2, 3))
    print(f"  kp={opt['kp']}  ki={opt['ki']}  -> {opt['rumbo_rms_deg']}° rms de rumbo", flush=True)
