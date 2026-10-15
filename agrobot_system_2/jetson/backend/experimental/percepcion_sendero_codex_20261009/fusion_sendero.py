"""Ground-camera projection and conservative fusion, without hardware IO.

Input: calibrated, undistorted image semantic labels from a separately trained
model. This module does not invent a segmentation network or metric depth.
Coordinate convention: x forward, y left, z up, camera pitch positive down.
"""
from dataclasses import dataclass
import math
import numpy as np

UNKNOWN,FREE,BLOCKED=0,1,2

@dataclass(frozen=True)
class Calibration:
    fx:float
    fy:float
    cx:float
    cy:float
    height:float
    camera_forward:float
    pitch_home_deg:float
    servo_home:float
    pitch_per_servo_deg:float # signed, measured, never inferred from name
    verified:bool=False

    def pitch(self,servo):
        return math.radians(self.pitch_home_deg+self.pitch_per_servo_deg*(servo-self.servo_home))

    def ground(self,u,v,servo):
        """Intersect rays with z=0. NaN for horizon/behind; no object height."""
        if not self.verified:raise ValueError('Calibration must be measured and verified')
        pitch=self.pitch(servo);dy=(np.asarray(v)-self.cy)/self.fy
        down=math.sin(pitch)+dy*math.cos(pitch)
        forward=math.cos(pitch)-dy*math.sin(pitch)
        with np.errstate(divide='ignore',invalid='ignore'):
            scale=self.height/down
            x=self.camera_forward+scale*forward
            y=-scale*(np.asarray(u)-self.cx)/self.fx
        valid=(down>.05)&(forward>0)&np.isfinite(x)&np.isfinite(y)
        return np.where(valid,x,np.nan),np.where(valid,y,np.nan)

class GroundFusion:
    """Current local ground grid. Lidar can block; it cannot clear low hazards.

    Label 1=path, 2=crop/raised bed/obstacle, 0=unknown. Shadow is not an
    obstacle class; classify its underlying surface or leave unknown.
    Ground intersections of raised objects are apparent locations, not depth.
    Caller must supply calibrated lower-bound clear distances for motion.
    """
    def __init__(self,res=.025,forward=2.,half_width=.75):
        self.res=res;self.half_width=half_width
        self.shape=(math.ceil(forward/res),math.ceil(2*half_width/res))

    def build(self,labels,confidence,calibration,servo,frame_age_s,
              servo_settled,quality_ok,lidar_points=(),max_age=.3):
        grid=np.zeros(self.shape,np.uint8)
        valid=calibration.verified and servo_settled and quality_ok and 0<=frame_age_s<=max_age
        reason='valid_image' if valid else 'unknown_image_or_calibration'
        if valid:
            v,u=np.indices(labels.shape)
            x,y=calibration.ground(u,v,servo)
            ok=np.isfinite(x)&np.isfinite(y)&(x>=0)&(x<self.shape[0]*self.res)&(abs(y)<self.half_width)&(confidence>=.85)
            ix=(x[ok]/self.res).astype(int);iy=((y[ok]+self.half_width)/self.res).astype(int)
            values=labels[ok]
            # np.maximum.at prevents a later path pixel clearing an obstacle.
            np.maximum.at(grid,(ix,iy),np.where(values==1,FREE,np.where(values==2,BLOCKED,UNKNOWN)))
        for x,y in lidar_points:
            if np.isfinite(x) and np.isfinite(y) and 0<=x<self.shape[0]*self.res and abs(y)<self.half_width:
                grid[int(x/self.res),int((y+self.half_width)/self.res)]=BLOCKED
        return {'grid':grid,'vision_valid':valid,'reason':reason,'metric_depth':False}

def speed_limit(clearance_lower_bound_m,latency_s,deceleration_mps2,margin_m=.08,maximum_mps=.2):
    """Solve v*t + v²/(2a) <= d-margin. None/unknown means no forward motion.
    clearance is from the robot's leading swept boundary, not camera origin.
    Deceleration must be measured; same acceleration setting is not evidence.
    """
    if clearance_lower_bound_m is None:return 0.
    if not all(math.isfinite(v) for v in (clearance_lower_bound_m,latency_s,deceleration_mps2,margin_m,maximum_mps)):
        return 0.
    if latency_s<0 or deceleration_mps2<=0 or margin_m<0 or maximum_mps<0:return 0.
    usable=max(0.,clearance_lower_bound_m-margin_m);a=deceleration_mps2
    return min(maximum_mps,max(0.,-a*latency_s+math.sqrt((a*latency_s)**2+2*a*usable)))
