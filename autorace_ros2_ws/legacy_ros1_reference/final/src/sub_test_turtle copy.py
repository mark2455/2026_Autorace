#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose


rospy.init_node("wego_node") 
def callback(data):
    print(f"sub_data:{data}")
rospy.Subscriber("/turtle1/pose",Pose,callback)
rospy.spin()

