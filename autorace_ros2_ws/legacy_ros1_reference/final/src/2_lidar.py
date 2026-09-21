#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import LaserScan
from math import *

class Webot_control:
    def __init__(self):
        rospy.init_node("lidar_node") # node 이름 정하기
        self.velocity_id_pub = rospy.Publisher("velocity_id",Int32, queue_size=1)
        rospy.Subscriber("/scan",LaserScan,self.laser_CB)
        self.rate = rospy.Rate(10) # 주기설정
        self.laser_msg = LaserScan()
        self.safe_range = 0.2 # 전방 장애물 있을 경우, 장애물 앞 몇 m에서 정지신호 보낼지 지정

    def laser_CB(self,msg):
        #  print(f"angle_min:{msg.angle_min}")
        #  print(f"angle_min_degree:{msg.angle_min*180/pi}")
        #  print(f"angle_max:{msg.angle_max}")
        #  print(f"angle_max_degree:{msg.angle_max*180/pi}")
        #  print(f"angle_increment:{msg.angle_increment}")
        #  print(f"angle_increment_degree:{msg.angle_increment*180/pi}")
        #  print(f"range_min:{msg.range_min}")
        #  print(f"range_max:{msg.range_max}")
        #for index, value in enumerate(msg.ranges):
            #if index < 107 or index > 1178:
            #    print(value)
         print(f"--------------------")
         print(len(msg.ranges))
         num = 0
         speed = 0
         
         degrees = [(msg.angle_min + msg.angle_increment * index)*180/pi for index, value in enumerate(msg.ranges)]

         for index, value in enumerate(msg.ranges):
            if 150 < abs(degrees[index]) and 0 < value <self.safe_range:
                num+=1
            else :
                pass

         if num > 10:
            speed = 0
            self.velocity_id_pub.publish(0)
            print("stop")
         else:
            speed = 1000
            self.velocity_id_pub.publish(1000)
            print("go")



def main():
    webot_control = Webot_control()
    rospy.spin()


if __name__=="__main__":
    main()
