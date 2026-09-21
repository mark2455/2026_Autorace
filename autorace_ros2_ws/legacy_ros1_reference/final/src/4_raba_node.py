#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from math import atan2, sqrt

class raba_control:
    def __init__(self):
        rospy.init_node("webot_node")
        self.steering_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.steering_msg = Float64()
        self.steering_angle = 0.0

        # Distance threshold to start avoiding obstacles
        self.distance_threshold = 0.5

        # Steering control limits
        self.steering_limit = 1.0

        # PID Controller parameters
        self.kp = 0.75
        self.ki = 0.000013566
        self.kd = 0.075
        self.integral = 0.0
        self.previous_error = 0.0

    def laser_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        min_distance = float('inf')  # 무한대로 초기화
        closest_point = None

        for point in points:
            distance = sqrt(point[0]**2 + point[1]**2)
            if distance < min_distance:
                min_distance = distance
                closest_point = point

        if closest_point is None:
            self.steering_angle = 0.5  # No obstacles detected, go straight
        else:
            desired_angle = atan2(closest_point[1], closest_point[0])  # 가장 가까운 장애물의 각도 계산
            error = desired_angle - self.steering_angle

            self.integral += error
            derivative = error - self.previous_error
            self.previous_error = error

            control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.steering_angle = self.limit_steering(control_output)

        self.steering_msg.data = 1 - self.steering_angle  # 원래는 self.steering_angle
        self.steering_pub.publish(self.steering_msg)

    def limit_steering(self, steering):
        if steering > self.steering_limit:
            return self.steering_limit
        elif steering < -self.steering_limit:
            return -self.steering_limit
        else:
            return steering

def main():
    raba_control = raba_control()

    rospy.Subscriber("/filter", PointCloud2, raba_control.laser_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        raba_control.steering_pub.publish(raba_control.steering_msg)
        rate.sleep()

if __name__ == "__main__":
    main()