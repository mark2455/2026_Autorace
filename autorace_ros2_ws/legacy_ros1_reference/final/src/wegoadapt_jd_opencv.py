#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float64
from math import *

rospy.init_node("speednode") 
pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
speed_msg = Float64()
pub2 = rospy.Publisher("/commands/servo/position",Float64,queue_size=1)
steer_msg = Float64()

while not rospy.is_shutdown():
    speed_msg.data = 1000
    print(f"pub_data:{speed_msg}")
    pub.publish(speed_msg) #3. 토픽을 보내는 송신 시점 설정

    steer_msg.data = 0.5
    print(f"pub_data:{steer_msg}")
    pub2.publish(steer_msg) #3. 토픽을 보내는 송신 시점 설정
    #rate.sleep()


