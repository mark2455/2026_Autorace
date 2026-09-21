#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from math import *

rospy.init_node("wego_pub_node") 
pub = rospy.Publisher("/counter",Int32,queue_size=1)
rate = rospy.Rate(2)

data = 0
while not rospy.is_shutdown():
    data=data+1
    print(f"pub_data:{data}")
    pub.publish(data) #3. 토픽을 보내는 송신 시점 설정
    rate.sleep()


