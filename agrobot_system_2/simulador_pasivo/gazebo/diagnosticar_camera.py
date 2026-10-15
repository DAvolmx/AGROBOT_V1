#!/usr/bin/env python3
"""Measure Gazebo RGB quality and corridor visibility from plants or furrows."""
import argparse, json, time
from pathlib import Path
import numpy as np
from percepcion_sendero import detectar_centros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Probe(Node):
    def __init__(self):
        super().__init__('agrobot_camera_quality')
        self.samples=[]
        self.create_subscription(Image, '/agrobot/camera/image_raw', self.cb, 2)
    def cb(self, msg):
        if msg.encoding.lower() not in ('rgb8','bgr8') or msg.step < msg.width*3:
            return
        arr=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding.lower() == 'bgr8': arr=arr[:,:,::-1]
        centers, source=detectar_centros(arr)
        self.samples.append({'std':float(arr.std()),'unique_colors':int(np.unique(arr.reshape(-1,3)[::max(1,msg.width*msg.height//512)],axis=0).shape[0]),'lane_rows':len(centers),'lane_source':source,'width':msg.width,'height':msg.height})

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--seconds',type=float,default=10); ap.add_argument('--out',default='/tmp/agrobot_camera_quality.json'); ap.add_argument('--movement-verified',action='store_true',help='Declare only after the camera has passed a separate moving-robot test'); args=ap.parse_args()
    rclpy.init(); node=Probe(); start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<args.seconds: rclpy.spin_once(node,timeout_sec=.1)
    finally:
        s=node.samples; n=len(s)
        usable=sum(x['std']>3 and x['unique_colors']>=16 for x in s); visible=sum(x['lane_rows']>=2 and x['std']>3 and x['unique_colors']>=16 for x in s)
        window_passed=bool(n and usable/n>=.9 and visible/n>=.5)
        report={'duration_s':args.seconds,'frames':n,'fps':round(n/args.seconds,2),'usable_rgb_pct':round(100*usable/max(1,n),1),'blank_frame_pct':round(100*(n-usable)/max(1,n),1),'lane_visible_pct':round(100*visible/max(1,n),1),'mean_lane_rows':round(float(np.mean([x['lane_rows'] for x in s])),2) if s else 0,'lane_source_pct':{source:round(100*sum(x['lane_source']==source and x['std']>3 and x['unique_colors']>=16 for x in s)/max(1,n),1) for source in ('vegetacion','surcos','sin_borde')},'resolution':[s[-1]['width'],s[-1]['height']] if s else None,'std_min':round(min(x['std'] for x in s),2) if s else None,'std_max':round(max(x['std'] for x in s),2) if s else None,'unique_colors_min':min((x['unique_colors'] for x in s),default=0),'capture_window_passed':window_passed,'movement_verified':args.movement_verified,'camera_approved_for_training':bool(window_passed and args.movement_verified)}
        Path(args.out).write_text(json.dumps(report,indent=2),encoding='utf-8'); print(json.dumps(report,indent=2))
        node.destroy_node(); rclpy.shutdown()
    if not report['camera_approved_for_training']: raise SystemExit('FAIL: capture window did not pass or a separate moving-camera test has not been verified')
if __name__=='__main__': main()
