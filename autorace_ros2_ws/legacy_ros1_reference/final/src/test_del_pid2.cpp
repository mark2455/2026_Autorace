#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <cmath>

ros::Publisher steering_pub;

// Distance threshold to start avoiding obstacles
double distance_threshold = 0.07; // Set your desired distance threshold here

// Steering control limits
double steering_limit = 0.7;
double straight_steering = 0.0;

// PID control parameters
double Kp = 0.5;  // Proportional gain
double Ki = 0.0005;  // Integral gain
double Kd = 0.05; // Derivative gain

double error_integral = 0.0; // Integral of error for the PID control
double prev_error = 0.0;     // Previous error for the PID control

// Function to limit the steering control within the specified range
double limitSteering(double steering)
{
  if (steering > steering_limit)
    return steering_limit;
  else if (steering < -steering_limit)
    return -steering_limit;
  else
    return steering;
}

// Function to calculate the distance to the left obstacle
double obstacleLeftDistance(const pcl::PointCloud<pcl::PointXYZ>& obstacle_cloud)
{
  double left_min_distance = std::numeric_limits<double>::max();

  for (int i = 0; i < obstacle_cloud.size(); ++i) {
    double distance = std::sqrt(std::pow(obstacle_cloud.points[i].x, 2) + std::pow(obstacle_cloud.points[i].y, 2));
    if (distance < left_min_distance && obstacle_cloud.points[i].x < 0) {
      left_min_distance = distance;
    }
  }

  return left_min_distance;
}

// Function to calculate the distance to the right obstacle
double obstacleRightDistance(const pcl::PointCloud<pcl::PointXYZ>& obstacle_cloud)
{
  double right_min_distance = std::numeric_limits<double>::max();

  for (int i = 0; i < obstacle_cloud.size(); ++i) {
    double distance = std::sqrt(std::pow(obstacle_cloud.points[i].x, 2) + std::pow(obstacle_cloud.points[i].y, 2));
    if (distance < right_min_distance && obstacle_cloud.points[i].x >= 0) {
      right_min_distance = distance;
    }
  }

  return right_min_distance;
}

void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
  pcl::fromROSMsg(*msg, obstacle_cloud);

  // Calculate the distance to the left obstacle
  double left_distance = obstacleLeftDistance(obstacle_cloud);

  // Calculate the distance to the right obstacle
  double right_distance = obstacleRightDistance(obstacle_cloud);

  // Calculate the error as the difference between the target distance and the actual distance
  double error = right_distance - left_distance;

  // Calculate the PID control output
  double control_output = Kp * error + Ki * error_integral + Kd * (error - prev_error);

  // Update the error integral and previous error for the next iteration
  error_integral += error;
  prev_error = error;

  // Calculate the steering angle based on the control output
  double steering_angle = limitSteering(control_output);

  // Publish the steering angle
  std_msgs::Float64 steering_msg;
  steering_msg.data = steering_angle;
  steering_pub.publish(steering_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_del");
  ros::NodeHandle nh;

  ros::Subscriber obstacle_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter", 10, obstacleCallback);
  steering_pub = nh.advertise<std_msgs::Float64>("/del", 10);

  ros::spin();
  
  error_integral = 0.0;
  prev_error = 0.0;

  return 0;
}
