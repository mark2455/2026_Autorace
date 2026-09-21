#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from math import *


class Webot_control:
    def __init__(self):
        rospy.init_node("velocity_node") # node 이름 정하기
        self.webot_ctrl_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1) # node 역할 정하기
        
        rospy.Subscriber("velocity_id",Int32,self.velocity_node)
        self.vel = Int32()



    def velocity_node(self,vel):
        print("vel",vel)
        if vel == Int32(0):
            self.webot_ctrl_pub.publish(0)
            print("stop")
        else:
            self.webot_ctrl_pub.publish(1000)
            print("go")



def main():
    velocity_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()

