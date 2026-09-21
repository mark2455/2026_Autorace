#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from math import *

class Webot_control:
    def __init__(self):
        rospy.init_node("webot_node")
        self.steering_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.steering_msg = Float64()
        self.steering_angle = 0.0

        # Distance threshold to start avoiding obstacles
        self.distance_threshold = 0.7

        # Steering control limits
        self.steering_limit = 1.0

        # PID Controller parameters
        self.kp = 0.75
        self.ki = 0.000013566
        self.kd = 0.075
        self.integral = 0.0
        self.previous_error = 0.0

    def laser_callback(self, msg):
        ranges = msg.ranges

        # Find the closest obstacle within the detectable range
        min_distance = min(ranges)

        error = 0.0
        if min_distance < self.distance_threshold:
            # Find the angle to the closest obstacle
            min_index = ranges.index(min_distance)
            angle_to_obstacle = msg.angle_min + min_index * msg.angle_increment
            desired_angle = atan2(sin(angle_to_obstacle), cos(angle_to_obstacle))  # Normalize angle to [-pi, pi]
            error = desired_angle - self.steering_angle

        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.steering_angle = self.limit_steering(control_output)

        self.steering_msg.data = self.steering_angle
        self.steering_pub.publish(self.steering_msg)

    def limit_steering(self, steering):
        if steering > self.steering_limit:
            return self.steering_limit
        elif steering < -self.steering_limit:
            return -self.steering_limit
        else:
            return steering

def main():
    webot_control = Webot_control()

    rospy.Subscriber("/filter", PointCloud2, webot_control.laser_callback)  # 수정된 부분

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        webot_control.steering_pub.publish(webot_control.steering_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
