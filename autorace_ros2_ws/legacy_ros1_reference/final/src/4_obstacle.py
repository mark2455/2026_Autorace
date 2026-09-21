#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import LaserScan
from math import *

class Obstacle_control:
    def __init__(self):
        rospy.init_node("webot_node") # node 이름 정하기
        self.webot_ctrl_pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=1) # node 역할 정하기
        rospy.Subscriber("steer_rabacon_id",Int32, self.laser_CB)
        rospy.Subscriber("/scan",LaserScan,self.laser_CB)
        self.rate = rospy.Rate(10) # 주기설정
        self.laser_msg = LaserScan()
        self.safe_range = 0.60     #강의에선 0.3인데, 우리것은 LiDAR 코앞에 갔다 대어도 0.3일 때 정지 안하는 문제가... 따라서 0.6정도 잡음.
        self.msg = Int32()

    def laser_CB(self,msg):
        if msg == Int32(1):
            print(f"--------------------")
            #print(len(msg.ranges))
            num = 0
            speed = 0
            
            degrees = [(msg.angle_min + msg.angle_increment * index)*180/pi for index, value in enumerate(msg.ranges)]

            for index, value in enumerate(msg.ranges):
                if 150 < abs(degrees[index]) and 0 < value <self.safe_range:
                    num+=1
                else :
                    pass

            if num > 10:
                self.webot_ctrl_pub.publish(0)
                print("stop")
            else:
                self.webot_ctrl_pub.publish(1000)   
                print("go")



def main():
    obstacle_control = Obstacle_control()
    rospy.spin()


if __name__=="__main__":
    main()
