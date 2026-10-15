#!/usr/bin/env python3
import sys,time
import rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
class Capture(Node):
 def __init__(self,path):
  super().__init__('agrobot_gazebo_capture'); self.path=path; self.done=False
  self.sub=self.create_subscription(Image,'/agrobot/camera/image_raw',self.cb,1)
 def cb(self,m):
  if self.done:return
  c=3 if m.encoding.lower() in ('rgb8','bgr8') else 1
  a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.width,c)
  if m.encoding.lower()=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
  if not cv2.imwrite(self.path,a):raise RuntimeError('imwrite failed')
  self.get_logger().info(f'saved {self.path} {m.width}x{m.height} {m.encoding}');self.done=True
def main():
 rclpy.init();n=Capture(sys.argv[1]);t=time.time()
 try:
  while rclpy.ok() and not n.done and time.time()-t<20:rclpy.spin_once(n,timeout_sec=.2)
  if not n.done:raise SystemExit('timeout waiting camera frame')
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
