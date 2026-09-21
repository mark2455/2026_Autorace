#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import LaserScan
from math import *

class Choice_control:
    def __init__(self):
        rospy.init_node("choice_node") # node 이름 정하기
        self.rabasteer_pub = rospy.Publisher("rabasteer",Int32, queue_size=1)
        if 1==1:
            self.rabasteer_pub.publish(0)
            print("rabasteer is on")

        self.ordisteer_pub = rospy.Publisher("ordisteer",Int32, queue_size=1)
        if 1==2:
            self.ordisteer_pub.publish(1)
            print("ordisteer is on")

def main():
    choice_control = Choice_control()
    rospy.spin()


if __name__=="__main__":
    main()
