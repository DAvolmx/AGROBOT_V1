import argparse
import json
from pathlib import Path
import time
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class Capture(Node):
    def __init__(self,out):
        super().__init__('agrobot_visual_photo_check');self.out=out
        self.frames=0;self.statistics=[];self.scan=None
        self.create_subscription(Image,'/agrobot/camera/image_raw',self.image,1)
        self.create_subscription(LaserScan,'/agrobot/scan',self.lidar,1)
    def image(self,msg):
        if msg.encoding.lower() not in ('rgb8','bgr8'):return
        rgb=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding.lower()=='bgr8':rgb=rgb[:,:,::-1]
        self.frames+=1
        PILImage.fromarray(rgb).save(self.out/'camera_fotografica.png')
        self.statistics.append({'std':float(rgb.std()),'width':msg.width,'height':msg.height})
    def lidar(self,msg):
        self.scan={'angle_min':msg.angle_min,'angle_increment':msg.angle_increment,
                   'range_min':msg.range_min,'range_max':msg.range_max,
                   'ranges':[float(x) if np.isfinite(x) else None for x in msg.ranges]}


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Capture(a.out);start=time.monotonic()
    try:
        while time.monotonic()-start<20:rclpy.spin_once(node,timeout_sec=.1)
        report={'frames':node.frames,'duration_s':20,'fps':node.frames/20,
                'rgb_nonblank':bool(node.statistics and min(s['std'] for s in node.statistics)>3),
                'statistics':node.statistics,'lidar':node.scan,'movement_tested':False}
        (a.out/'visual_check.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('statistics','lidar')}))
        if not report['rgb_nonblank']:raise RuntimeError('No usable RGB frames')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
