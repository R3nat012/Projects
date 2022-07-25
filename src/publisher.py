#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64


if __name__ == '__main__':
  
  rospy.init_node('Publisher')
  pub = rospy.Publisher('counter', Float64, queue_size = 1)
  rate = rospy.Rate(20) # 20 Hz

  count = 0
  while not rospy.is_shutdown():
    
    var = Float64()
    var.data = count
    rospy.loginfo(str(var.data))
    pub.publish(var)
    count = count + 1
    rate.sleep
