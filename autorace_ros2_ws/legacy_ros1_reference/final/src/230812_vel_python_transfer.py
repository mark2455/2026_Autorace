#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2

import math

steering_pub = None
steering_msg = Float64()
steering_angle = 0.0

distance_threshold = 0.7
steering_limit = 0.7
straight_steering = 0.0
kp = 0.75
ki = 0.000013566
kd = 0.075
integral = 0.0
previous_error = 0.0

def limit_steering(steering):
    if steering > steering_limit:
        return steering_limit
    elif steering < -steering_limit:
        return -steering_limit
    else:
        return steering

class WebotControl:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.webot_ctrl_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.obstacle_left_sub = rospy.Subscriber('/filter_L', PointCloud2, self.obstacle_left_callback)
        self.obstacle_right_sub = rospy.Subscriber('/filter_R', PointCloud2, self.obstacle_right_callback)

    def obstacle_left_callback(self, msg):
        obstacle_cloud = pcl.PointCloud()
        pcl.fromROSMsg(msg, obstacle_cloud)

        # Calculate the average distance from the origin to the obstacle points
        min_distance = float('inf')
        for point in obstacle_cloud:
            distance = math.sqrt(point.x ** 2 + point.y ** 2)
            if distance < min_distance:
                min_distance = distance

        # Calculate the steering angle based on the distance to the obstacle
        error = 0.0
        if min_distance < distance_threshold:
            desired_angle = math.atan2(point.y, point.x)
            error = desired_angle - steering_angle

        # Update the integral term
        global integral
        integral += error

        # Update the derivative term
        global previous_error
        derivative = error - previous_error
        previous_error = error

        # Calculate the steering control output using PID equation
        control_output = kp * error + ki * integral + kd * derivative

        # Limit the steering control within the specified range
        # global steering_angle
        steering_angle = limit_steering(control_output)

        # Publish the steering angle
        steering_msg.data = steering_angle
        steering_pub.publish(steering_msg)

    def obstacle_right_callback(self, msg):
        obstacle_cloud = pcl.PointCloud()
        pcl.fromROSMsg(msg, obstacle_cloud)

        # Calculate the average distance from the origin to the obstacle points
        min_distance = float('inf')
        for point in obstacle_cloud:
            distance = math.sqrt(point.x ** 2 + point.y ** 2)
            if distance < min_distance:
                min_distance = distance

        # Calculate the steering angle based on the distance to the obstacle
        error = 0.0
        if min_distance < distance_threshold:
            desired_angle = math.atan2(-point.y, -point.x)
            error = desired_angle - steering_angle

        # Update the integral term
        global integral
        integral += error

        # Update the derivative term
        global previous_error
        derivative = error - previous_error
        previous_error = error

        # Calculate the steering control output using PID equation
        control_output = kp * error + ki * integral + kd * derivative

        # Limit the steering control within the specified range
        #global steering_angle
        steering_angle = limit_steering(control_output)

        steering_angle = -steering_angle

        # Publish the steering angle
        steering_msg.data = steering_angle
        steering_pub.publish(steering_msg)

    def run(self):
        rospy.loginfo("Current time: %f", rospy.Time.now().to_sec())
        # Add any continuous control logic here if needed

def main():
    rospy.init_node('test_del')
    global steering_pub
    steering_pub = rospy.Publisher('steer_msg', Float64, queue_size=10)

    webot_control = WebotControl()

    while not rospy.is_shutdown():
        webot_control.run()
        rospy.spin()

if __name__ == '__main__':
    main()