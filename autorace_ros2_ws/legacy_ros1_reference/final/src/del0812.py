#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from math import *

class Webot_control:
    def __init__(self):
        rospy.init_node("webot_node")
        self.webot_ctrl_pub = rospy.Publisher("/commands/servo/position",Float64,queue_size=1)
        rospy.Subscriber("/scan",LaserScan,self.laser_CB)
        self.rate = rospy.Rate(10)
        self.laser_msg = LaserScan()
        self.safe_range = 0.30
        self.detect_degree = 90
        self.detect_range = 0.6
        self.sensitivity = 10
        
    def laser_CB(self,msg):
        num = 0    
        index_list=[]
        degree_list=[]
        left_degree=[]
        right_degree=[]
        degrees = [(msg.angle_min + msg.angle_increment * index)*180/pi for index, value in enumerate(msg.ranges)]
        for index, value in enumerate(msg.ranges):
            if self.detect_degree < abs(degrees[index]) and 0 < value <self.detect_range:
                degree_list.append(degrees[index])
        for degree in degree_list:
            if degree > 0:
                right_degree.append(degree)
            else:
                left_degree.append(degree)
        num_right_degree = len(right_degree)
        num_left_degree = len(left_degree)

        if num_right_degree < self.sensitivity:
            num_right_degree = 0
        if num_left_degree < self.sensitivity:
            num_left_degree = 0

        # Calculate average distance for left and right obstacles
        avg_left_distance = sum([msg.ranges[int((self.detect_degree + degree) * len(msg.ranges) / 360)] for degree in left_degree]) / num_left_degree if num_left_degree > 0 else 0
        avg_right_distance = sum([msg.ranges[int((self.detect_degree + degree) * len(msg.ranges) / 360)] for degree in right_degree]) / num_right_degree if num_right_degree > 0 else 0

        # Set steer based on obstacle distances
        if avg_left_distance == 0 and avg_right_distance == 0:
            steer = 0.5  # No obstacle detected, go straight
        elif avg_left_distance < avg_right_distance:
            steer = 0     # Left obstacle is closer, steer right
        else:
            steer = 1     # Right obstacle is closer, steer left

        self.webot_ctrl_pub.publish(steer)

def main():
    webot_control = Webot_control()
    rospy.spin()

if __name__=="__main__":
    main()
