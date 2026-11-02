"""VIGÍA DE LA POSE (2026-11-02): la autonomía navega con una pose que no salta aunque el SLAM o el EKF salten.

Fallos reales que motivan esto:
  · 27-oct: el reloj de pared saltó +711 s (Jetson sin RTC) y el EKF dio un pico de vx = −29,7 m/s: la pose del mapa saltó 30 y 24 m en plena misión y las plantas
    quedaron mal ubicadas. El robot siguió decidiendo con esa pose.
  · 30-sep: con el LIDAR caído el EKF extrapoló la última velocidad (varianza de vx disparada) y estiró el mapa a 9,7 km.

Idea: la pose del mapa es `map->odom ∘ odom->base_link`. La odometría (EKF, marco odom) es continua a corto plazo; el SLAM corrige `map->odom` (el ANCLA) poco a poco.
  · Mientras el ancla del SLAM cambia poco de un instante a otro (cierres de bucle normales: 25–35 cm medidos en interior), se sigue al SLAM («ok»).
  · Si el ancla da un SALTO (> `salto_m` o `salto_deg`) se CONSERVA la anterior y se sirve «ancla vieja ∘ odometría» («anclada»): estima a ciegas, pero continua.
    Vuelve a «ok» cuando el SLAM regresa cerca del ancla conservada, o cuando el ancla nueva se queda QUIETA `aceptar_s` y está a menos de `aceptar_max_m`
    (el SLAM se reubicó de verdad: «reubicada»).
  · Un salto de la ODOMETRÍA (velocidad imposible para este chasis) se absorbe: la pose servida no se mueve en ese instante.
  · Anclado demasiado tiempo o demasiados metros, o con el EKF sin medidas, la pose ya no es de fiar: «perdida». Quien llama para la autonomía, pide ZUPT y avisa;
    se sale de «perdida» por las mismas reglas de vuelta a «ok» (el robot se recupera solo si el SLAM vuelve) o con `reubicar()` del operador.
  · Tras un salto del reloj (`sospechar()`) el umbral de salto baja un rato: es justo cuando el SLAM suele saltar.

Lógica pura, sin ROS ni reloj propio: `paso(mono, pose_mapa, pose_odom, var_vx)` recibe las dos poses ({x, y, theta_deg}, cualquiera puede ser None) y devuelve
el estado y la pose robusta. Ángulos en grados con la convención de ROS (+ = izquierda)."""
from __future__ import annotations

import math

OK, ANCLADA, PERDIDA, SIN_DATOS = "ok", "anclada", "perdida", "sin_datos"


def _ang(a: float) -> float:
    """Normaliza radianes a (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _compone(t, o):
    """t ∘ o: aplica la transformación t=(x, y, th) a la pose o=(x, y, th). Radianes."""
    c, s = math.cos(t[2]), math.sin(t[2])
    return (t[0] + c * o[0] - s * o[1], t[1] + s * o[0] + c * o[1], _ang(t[2] + o[2]))


def _ancla(m, o):
    """La transformación t tal que t ∘ o = m (el map->odom que implican las dos poses)."""
    th = _ang(m[2] - o[2])
    c, s = math.cos(th), math.sin(th)
    return (m[0] - (c * o[0] - s * o[1]), m[1] - (s * o[0] + c * o[1]), th)


def _tupla(p):
    if not p:
        return None
    try:
        return (float(p["x"]), float(p["y"]), math.radians(float(p["theta_deg"])))
    except (KeyError, TypeError, ValueError):
        return None


def _distancia(a, b):
    """(metros, grados) entre dos poses."""
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(math.degrees(_ang(a[2] - b[2])))


class VigiaPose:
    def __init__(self, *, salto_m: float = 0.50, salto_deg: float = 25.0, salto_sospecha_m: float = 0.15, salto_sospecha_deg: float = 8.0,
                 resync_m: float = 0.25, resync_deg: float = 10.0, aceptar_s: float = 10.0, aceptar_max_m: float = 1.0, quieto_m: float = 0.05,
                 max_anclada_m: float = 6.0, max_anclada_s: float = 180.0, v_imposible_mps: float = 1.2, w_imposible_dps: float = 400.0,
                 odom_salto_m: float = 0.5, ekf_var_max: float = 1.0, ekf_var_s: float = 2.0, odom_vieja_s: float = 1.5):
        self.salto_m, self.salto_deg = salto_m, salto_deg
        self.salto_sospecha_m, self.salto_sospecha_deg = salto_sospecha_m, salto_sospecha_deg
        self.resync_m, self.resync_deg = resync_m, resync_deg
        self.aceptar_s, self.aceptar_max_m, self.quieto_m = aceptar_s, aceptar_max_m, quieto_m
        self.max_anclada_m, self.max_anclada_s = max_anclada_m, max_anclada_s
        self.v_imposible, self.w_imposible = v_imposible_mps, w_imposible_dps
        self.odom_salto_m = odom_salto_m
        self.ekf_var_max, self.ekf_var_s = ekf_var_max, ekf_var_s
        self.odom_vieja_s = odom_vieja_s
        # estado
        self.estado = SIN_DATOS
        self.motivo = "sin odometría todavía"
        self.ancla = None                  # map->odom que se está usando (x, y, th)
        self._odom_prev = None
        self._t_prev = None
        self._t_odom = None                # último instante con odometría válida
        self.pose = None                   # última pose servida (x, y, th)
        self._sospecha_hasta = -1.0
        self._anclada_desde = None
        self._anclada_m = 0.0
        self._cand = None                  # ancla candidata del SLAM mientras se espera a aceptarla
        self._cand_desde = None
        self._ekf_mal_desde = None
        self.eventos: list[dict] = []      # transiciones y saltos absorbidos (los recoge quien llama)
        self.cuenta = {"saltos_slam": 0, "saltos_odom": 0, "reubicadas": 0, "perdidas": 0, "resync": 0}

    # ------------------------------------------------------------------ API
    def sospechar(self, mono: float, segundos: float = 15.0):
        """Avisa de un salto del reloj: durante `segundos` cualquier salto del ancla mayor que `salto_sospecha_m` se trata como salto."""
        self._sospecha_hasta = max(self._sospecha_hasta, mono + segundos)

    def reubicar(self, mono: float, pose_mapa: dict):
        """El operador fija la pose (pose correcta en el mapa). Sale de «anclada»/«perdida»."""
        m = _tupla(pose_mapa)
        if m is None or self._odom_prev is None:
            return False
        self.ancla = _ancla(m, self._odom_prev)
        self._a_ok(mono, "reubicada por el operador")
        self.pose = m
        return True

    def pose_dict(self) -> dict | None:
        if self.pose is None:
            return None
        return {"x": self.pose[0], "y": self.pose[1], "theta_deg": math.degrees(self.pose[2]) % 360.0,
                "fuente": "slam" if self.estado == OK else "odometria", "estado_localizacion": self.estado}

    def autonomia_permitida(self) -> bool:
        return self.estado in (OK, ANCLADA)

    def snapshot(self) -> dict:
        return {"estado": self.estado, "motivo": self.motivo, "anclada_m": round(self._anclada_m, 2), **self.cuenta}

    # ------------------------------------------------------------------ núcleo
    def paso(self, mono: float, pose_mapa: dict | None, pose_odom: dict | None, var_vx: float | None = None) -> dict:
        m, o = _tupla(pose_mapa), _tupla(pose_odom)
        dt = None if self._t_prev is None else max(1e-3, mono - self._t_prev)
        self._t_prev = mono

        # 1) odometría: continua salvo saltos imposibles, que se absorben
        if o is not None:
            if self._odom_prev is not None and self.ancla is not None and dt is not None:
                d_m, d_deg = _distancia(o, self._odom_prev)
                dt_odom = max(dt, mono - (self._t_odom or mono))
                if d_m > self.odom_salto_m or d_m / dt_odom > self.v_imposible or d_deg / dt_odom > self.w_imposible:
                    servida = self.pose if self.pose is not None else _compone(self.ancla, self._odom_prev)
                    self.ancla = _ancla(servida, o)          # la pose servida se queda donde estaba
                    self.cuenta["saltos_odom"] += 1
                    self._evento(mono, "salto_odometria", m=round(d_m, 2), deg=round(d_deg, 1))
                    if self.estado == OK:
                        self._a_anclada(mono, f"la odometría saltó {d_m:.2f} m")
                elif self.estado == ANCLADA:
                    self._anclada_m += d_m
            self._odom_prev, self._t_odom = o, mono
        elif self._t_odom is None or mono - self._t_odom > self.odom_vieja_s:
            if self.estado != SIN_DATOS and self.estado != PERDIDA and self.ancla is not None:
                self._a_perdida(mono, "sin odometría")
            elif self.ancla is None:
                self.estado, self.motivo = SIN_DATOS, "sin odometría"
            return self._salida()

        # 2) EKF extrapolando (varianza de vx disparada): la odometría tampoco vale
        if var_vx is not None and var_vx > self.ekf_var_max:
            self._ekf_mal_desde = mono if self._ekf_mal_desde is None else self._ekf_mal_desde
            if mono - self._ekf_mal_desde >= self.ekf_var_s and self.estado != PERDIDA:
                self._a_perdida(mono, f"EKF sin medidas (varianza vx {var_vx:.2f})")
        else:
            self._ekf_mal_desde = None

        # 3) ancla del SLAM
        if self.ancla is None:
            if m is None or self._odom_prev is None:
                return self._salida()
            self.ancla = _ancla(m, self._odom_prev)
            self._a_ok(mono, "primera pose")
        elif m is not None and self._odom_prev is not None:
            a_slam = _ancla(m, self._odom_prev)
            serv = _compone(self.ancla, self._odom_prev)
            dif_m, dif_deg = _distancia(_compone(a_slam, self._odom_prev), serv)
            sospecha = mono < self._sospecha_hasta
            lim_m = self.salto_sospecha_m if sospecha else self.salto_m
            lim_deg = self.salto_sospecha_deg if sospecha else self.salto_deg
            if self.estado == OK:
                if dif_m > lim_m or dif_deg > lim_deg:
                    self.cuenta["saltos_slam"] += 1
                    self._evento(mono, "salto_slam", m=round(dif_m, 2), deg=round(dif_deg, 1), sospecha=sospecha)
                    self._a_anclada(mono, f"el SLAM saltó {dif_m:.2f} m / {dif_deg:.0f}°")
                    self._cand, self._cand_desde = a_slam, mono
                else:
                    self.ancla = a_slam                       # se sigue al SLAM (correcciones normales)
            elif self._ekf_mal_desde is None:                 # anclada o perdida: ¿vuelve el SLAM?
                if dif_m <= self.resync_m and dif_deg <= self.resync_deg:
                    self.ancla = a_slam
                    self.cuenta["resync"] += 1
                    self._a_ok(mono, "el SLAM volvió a la pose conservada")
                else:
                    if self._cand is None or _distancia(_compone(a_slam, self._odom_prev), _compone(self._cand, self._odom_prev))[0] > self.quieto_m:
                        self._cand, self._cand_desde = a_slam, mono    # el SLAM sigue moviéndose: se vuelve a esperar
                    elif mono - self._cand_desde >= self.aceptar_s and dif_m <= self.aceptar_max_m and dif_deg <= 3 * self.resync_deg:
                        self.ancla = a_slam
                        self.cuenta["reubicadas"] += 1
                        self._a_ok(mono, f"el SLAM se reubicó ({dif_m:.2f} m) y se mantuvo {self.aceptar_s:.0f} s")

        # 4) demasiado tiempo / distancia a ciegas
        if self.estado == ANCLADA:
            if self._anclada_m > self.max_anclada_m:
                self._a_perdida(mono, f"{self._anclada_m:.1f} m sin SLAM fiable")
            elif mono - self._anclada_desde > self.max_anclada_s:
                self._a_perdida(mono, f"{mono - self._anclada_desde:.0f} s sin SLAM fiable")

        if self.ancla is not None and self._odom_prev is not None:
            self.pose = _compone(self.ancla, self._odom_prev)
        return self._salida()

    # ------------------------------------------------------------------ transiciones
    def _evento(self, mono, tipo, **datos):
        self.eventos.append({"t": round(mono, 2), "tipo": tipo, **datos})
        del self.eventos[:-50]

    def _cambiar(self, mono, estado, motivo):
        if estado != self.estado:
            self._evento(mono, "estado", de=self.estado, a=estado, motivo=motivo)
        self.estado, self.motivo = estado, motivo

    def _a_ok(self, mono, motivo):
        self._anclada_desde, self._anclada_m, self._cand, self._cand_desde = None, 0.0, None, None
        self._cambiar(mono, OK, motivo)

    def _a_anclada(self, mono, motivo):
        if self.estado != ANCLADA:
            self._anclada_desde, self._anclada_m = mono, 0.0
        self._cambiar(mono, ANCLADA, motivo)

    def _a_perdida(self, mono, motivo):
        if self.estado != PERDIDA:
            self.cuenta["perdidas"] += 1
        self._cambiar(mono, PERDIDA, motivo)

    def _salida(self) -> dict:
        return {"estado": self.estado, "motivo": self.motivo, "pose": self.pose_dict()}
