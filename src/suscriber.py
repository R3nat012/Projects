#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64

def cb_function(msg):
  
  rospy.loginfo(str(msg.data))


if __name__ =='__main__':

  rospy.init_node('suscriber')
  rospy.Subscriber('counter', Float64, cb_function)
  rospy.spin()
