#!/usr/bin/env python

import roslib
import rospy
import socket
import time
import math
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

x_current = 0
y_current = 0
theta_current = 0

def get_current_position(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.position.x)
  x_current = data.pose.position.x
  y_current = data.pose.position.y
  # theta_current = data.theta


if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    pose_sub = rospy.Subscriber('/vrpn_client_node/vehicle/pose',PoseStamped,get_current_position)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass