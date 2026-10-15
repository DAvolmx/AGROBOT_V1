"""Manual demo command validation. Values are transport references, not measured speed."""
import math
V_SCALE = .55
W_SCALE = 1.5

def finite(value, low, high):
    if isinstance(value, bool):
        raise ValueError('valor booleano no permitido')
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError('orden fuera de rango')
    return value

def command(v, w, duty=100):
    v, w, duty = finite(v,-1,1), finite(w,-1,1), finite(duty,0,100)
    v *= duty/100
    if abs(v) < .004:
        return 0., 0.
    return max(-.35,min(.60,v))*V_SCALE, (0. if v < 0 else w*W_SCALE)

def reference(v,w):
    v,w = finite(v,-100,100), finite(w,-100,100)
    return command(max(-1,min(1,v/V_SCALE)), max(-1,min(1,w/W_SCALE)))
