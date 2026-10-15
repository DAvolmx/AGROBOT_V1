#!/usr/bin/env python3
"""Smoke test of Gazebo ROS sensor streams and wheel-drive response."""
import argparse,json,math,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,LaserScan
from ros_gz_interfaces.msg import Contacts
SIDES=('front_left','front_right','rear_left','rear_right')
class Probe(Node):
 def __init__(self,duration):
  super().__init__('agrobot_gazebo_probe');self.duration=duration;self.start=time.monotonic()
  self.count={'camera':0,'scan':0,'odom':0,**{s:0 for s in SIDES}}
  self.first_pose=None;self.last_pose=None;self.min_range=float('inf');self.image=None
  self.cmd=self.create_publisher(Twist,'/agrobot/cmd_vel',10)
  self.create_subscription(Image,'/agrobot/camera/image_raw',self.on_image,1)
  self.create_subscription(LaserScan,'/agrobot/scan',self.on_scan,1)
  self.create_subscription(Odometry,'/agrobot/odom',self.on_odom,10)
  for side in SIDES:self.create_subscription(Contacts, f'/agrobot/contact/{side}', lambda m,s=side:self.on_contact(s),10)
 def on_image(self,m):self.count['camera']+=1;self.image=(m.width,m.height,m.encoding)
 def on_scan(self,m):
  self.count['scan']+=1
  vals=[r for r in m.ranges if math.isfinite(r) and r>0]
  if vals:self.min_range=min(self.min_range,min(vals))
 def on_odom(self,m):
  self.count['odom']+=1;p=m.pose.pose.position
  if self.first_pose is None:self.first_pose=(p.x,p.y)
  self.last_pose=(p.x,p.y)
 def on_contact(self,s):self.count[s]+=1
 def command(self,t):
  msg=Twist(); elapsed=time.monotonic()-self.start
  if elapsed<3.0:msg.linear.x=.12
  elif elapsed<5.0:msg.linear.x=.08;msg.angular.z=.22
  elif elapsed<7.0:msg.linear.x=.08;msg.angular.z=-.22
  self.cmd.publish(msg)
def main():
 p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=9);p.add_argument('--salida',default='/tmp/agrobot_probe.json');a=p.parse_args()
 rclpy.init();n=Probe(a.duration);timer=n.create_timer(.1,lambda:n.command(0))
 try:
  while rclpy.ok() and time.monotonic()-n.start<a.duration:rclpy.spin_once(n,timeout_sec=.05)
 finally:
  n.cmd.publish(Twist());rclpy.spin_once(n,timeout_sec=.15)
  dist=0
  if n.first_pose and n.last_pose:dist=math.hypot(n.last_pose[0]-n.first_pose[0],n.last_pose[1]-n.first_pose[1])
  report={'wall_duration_s':a.duration,'messages':n.count,'camera_last_frame':n.image,'odom_displacement_m':round(dist,3),'minimum_lidar_range_m':None if not math.isfinite(n.min_range) else round(n.min_range,3)}
  open(a.salida,'w').write(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
