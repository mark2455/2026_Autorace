#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32


rospy.init_node("wego_sub_node") 
def callback(data):
    print(f"sub_data:{data}")
rospy.Subscriber("/counter",Int32,callback)
rospy.spin()

