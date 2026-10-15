"""Seguimiento puro con progreso local para rutas densas (m, rad, s)."""
import math
import numpy as np


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SeguidorSendero:
    """Pure pursuit que evita saltar a otra rama cercana de un sendero.

    Mantiene el avance por arco y limita la proyección a una ventana local.
    `route` es la línea central estimada/mapa; no consulta verdad del simulador.
    """
    def __init__(self, route, closed=True, progress_m=None, backward_m=.15):
        self.route = np.asarray(route, dtype=float).reshape(-1, 2)
        if len(self.route) < 2:
            raise ValueError('route requires at least two points')
        self.closed = bool(closed)
        self.points = self.route if (not closed or np.linalg.norm(self.route[0]-self.route[-1]) < 1e-9) else np.vstack((self.route, self.route[0]))
        self.lengths = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.cumulative = np.r_[0., np.cumsum(self.lengths)]
        self.total = float(self.cumulative[-1])
        if self.total <= 1e-8:
            raise ValueError('route has zero length')
        self.progress = None if progress_m is None else float(progress_m)
        self.backward_m = float(backward_m)

    def _project(self, xy, forward_window):
        xy = np.asarray(xy, dtype=float)
        best = None
        n = len(self.lengths)
        for i, length in enumerate(self.lengths):
            if length <= 1e-9:
                continue
            a, b = self.points[i], self.points[i+1]
            t = float(np.clip(np.dot(xy-a, b-a)/(length*length), 0., 1.))
            s = float(self.cumulative[i] + t*length)
            if self.progress is not None and self.closed:
                s += round((self.progress-s)/self.total)*self.total
            elif self.progress is not None:
                s = float(np.clip(s, self.progress-self.backward_m, self.progress+forward_window))
            if self.progress is not None and self.closed and not (self.progress-self.backward_m <= s <= self.progress+forward_window):
                continue
            point = a + t*(b-a)
            dist2 = float(np.dot(point-xy, point-xy))
            # Tiny forward tie-breaker avoids stalling at segment boundaries.
            score = dist2 - 1e-8*(s if self.progress is None else s-self.progress)
            if best is None or score < best[0]:
                best = (score, s)
        if best is None:
            return self.progress if self.progress is not None else 0.
        return max(self.progress-self.backward_m, best[1]) if self.progress is not None else best[1]

    def _point_at(self, s):
        if self.closed:
            s = s % self.total
        else:
            s = float(np.clip(s, 0., self.total))
        i = min(int(np.searchsorted(self.cumulative, s, side='right')-1), len(self.lengths)-1)
        length = self.lengths[i]
        f = 0. if length <= 1e-9 else (s-self.cumulative[i])/length
        return self.points[i] + f*(self.points[i+1]-self.points[i])

    def orden(self, pose, speed=.2, lookahead=.4, gain=1., omega_max=1.2, lateral_accel=.22):
        x, y, yaw = map(float, pose)
        self.progress = self._project((x, y), max(1.0, 2.0*speed+lookahead))
        target = self._point_at(self.progress + lookahead)
        alpha = _wrap(math.atan2(target[1]-y, target[0]-x)-yaw)
        curvature = 2.0*math.sin(alpha)/max(lookahead, .08)
        safe_speed = math.sqrt(max(lateral_accel, .01)/max(abs(curvature), 1e-6))
        v = min(float(speed), safe_speed)/(1.0+.8*abs(alpha))
        omega = float(np.clip(gain*curvature*v, -omega_max, omega_max))
        return v, omega
