#!/usr/bin/env python2

import rospy
import numpy as np
import math as m
from nav_msgs.msg import Odometry

global X
global Q
X=[0,0,0]
Q=[0,0,0,0]

def function(msg):
  X[0]=msg.pose.pose.position.x
  X[1]=msg.pose.pose.position.y
  X[2]=msg.pose.pose.position.z
  Q[0]=msg.pose.pose.orientation.x
  Q[1]=msg.pose.pose.orientation.y
  Q[2]=msg.pose.pose.orientation.z
  Q[3]=msg.pose.pose.orientation.w

if __name__ == "__main__":
  rospy.init_node('suscriber')
  rospy.Subscriber('odom', Odometry,function)
  fxcurrent = open("/tmp/xcurrent.txt", "w")
  rate = rospy.Rate(2)
 
  while not rospy.is_shutdown():
    w = Q[3]
    ex = Q[0]
    ey = Q[1]
    ez = Q[2]
    
    R = np.array([[2*(w**2 + ex**2)-1, 2*(ex*ey-w*ez), 2*(ex*ez + w*ey)],
             [2*(ex*ey + w*ez), 2*(w**2 + ey**2)-1, 2*(ey*ez - w*ex)],
             [2*(ex*ez-w*ey), 2*(ey*ez+w*ex),2*(w**2 + ez**2)-1]])
    ang = m.atan2(R[1,0],R[0,0])

    pose= np.array([X[0], X[1], ang])
    print('a')
    fxcurrent.write(str(pose)+'\n')
    rate.sleep()

fxcurrent.close()

