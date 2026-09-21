#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import *

rospy.init_node("wego_pub_node") 
pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)
rate = rospy.Rate(1)
data = Twist()

while not rospy.is_shutdown():
    data.angular.z = 1
    print(f"pub_data:{data}")
    pub.publish(data) #3. 토픽을 보내는 송신 시점 설정
    rate.sleep()


