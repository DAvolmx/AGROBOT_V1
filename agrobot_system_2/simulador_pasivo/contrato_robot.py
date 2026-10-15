"""Command mapping; no hidden simulator state enters controller. SI units."""
import json,math
from pathlib import Path
CONFIG=json.loads(Path(__file__).with_name('contrato_robot.json').read_text())
def targets(v,w,wheelbase=None,front_outer=None,rear_outer=None,tire_width=None):
 c=CONFIG
 L=c['wheelbase_m'] if wheelbase is None else wheelbase
 bf=(c['front_outer_m'] if front_outer is None else front_outer)-(c['tire_width_m'] if tire_width is None else tire_width)
 br=(c['rear_outer_m'] if rear_outer is None else rear_outer)-(c['tire_width_m'] if tire_width is None else tire_width)
 if not all(math.isfinite(x) for x in (v,w,L,bf,br)) or min(L,bf,br)<=0:raise ValueError('invalid command or geometry')
 v=max(-c['v_scale_mps'],min(c['v_scale_mps'],v))
 k=math.tan(math.radians(c['steer_limit_deg']))/L
 w=max(-abs(v)*k,min(abs(v)*k,w))
 delta=math.atan(L*w/v) if abs(v)>.001 else 0
 front=v/math.cos(delta)
 a=[v-w*br/2,v+w*br/2,front-w*bf/2,front+w*bf/2]
 scale=max(c['v_scale_mps'],*(abs(x) for x in a))
 factor=c['v_scale_mps']/scale
 return v*factor,w*factor,[x*factor for x in a]
