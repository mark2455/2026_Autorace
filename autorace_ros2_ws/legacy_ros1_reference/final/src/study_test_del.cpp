#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <cmath>

ros::Publisher steering_pub;
std_msgs::Float64 steering_msg;
double steering_angle = 0.0;

// Distance threshold to start avoiding obstacles
double distance_threshold = 0.7; // Set your desired distance threshold here

// Steering control limits
double steering_limit = 0.7;
double straight_steering = 0.0;

// PID Controller parameters
double kp = 0.75; // Proportional gain
double ki = 0.000013566; // Integral gain
double kd = 0.075; // Derivative gain
double integral = 0.0;
double previous_error = 0.0;

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

void obstacleLeftCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
  pcl::fromROSMsg(*msg, obstacle_cloud);

  // Calculate the average distance from the origin to the obstacle points
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < obstacle_cloud.size(); ++i) {
    double distance = std::sqrt(std::pow(obstacle_cloud.points[i].x, 2) + std::pow(obstacle_cloud.points[i].y, 2));
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  // Calculate the steering angle based on the distance to the obstacle
  double error = 0.0;
  if (min_distance < distance_threshold) {
    // If the distance is below the threshold, steer away from the obstacle
    double desired_angle = std::atan2(obstacle_cloud.points[0].y, obstacle_cloud.points[0].x); 
    error = desired_angle - steering_angle;
  }

  // Update the integral term
  integral += error;

  // Update the derivative term
  double derivative = error - previous_error;
  previous_error = error;

  // Calculate the steering control output using PID equation
  double control_output = kp * error + ki * integral + kd * derivative;

  // Limit the steering control within the specified range
  steering_angle = limitSteering(control_output) ;


  // Publish the steering angle
  steering_msg.data = steering_angle ;
  steering_pub.publish(steering_msg);
}

void obstacleRightCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
  pcl::fromROSMsg(*msg, obstacle_cloud);

  // Calculate the average distance from the origin to the obstacle points
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < obstacle_cloud.size(); ++i) {
    double distance = std::sqrt(std::pow(obstacle_cloud.points[i].x, 2) + std::pow(obstacle_cloud.points[i].y, 2));
    if (distance < min_distance) {
      min_distance = distance; 
    }
  }

  // Calculate the steering angle based on the distance to the obstacle
  double error = 0.0;
  if (min_distance < distance_threshold) {
    // If the distance is below the threshold, steer away from the obstacle
    double desired_angle = std::atan2(-obstacle_cloud.points[0].y, -obstacle_cloud.points[0].x);
   error = desired_angle - steering_angle;
  }

  // Update the integral term
  integral += error;

  // Update the derivative term
  double derivative = error - previous_error;
  previous_error = error;

  // Calculate the steering control output using PID equation
  double control_output = kp * error + ki * integral + kd * derivative;

  // Limit the steering control within the specified range
  steering_angle = limitSteering(control_output);
  
  steering_angle = -steering_angle;

  // Publish the steering angle
  steering_msg.data = steering_angle;
  steering_pub.publish(steering_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_del");
  ros::NodeHandle nh;

  ros::Subscriber obstacle_left_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter_L", 10, obstacleLeftCallback);
  ros::Subscriber obstacle_right_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter_R", 10, obstacleRightCallback);
  steering_pub = nh.advertise<std_msgs::Float64>("steer_msg", 10);

  // 데이터를 받지 못해도 종료되지 않도록 루프 생성
   while (ros::ok())
   {
       ros::spinOnce();
   }

  return 0;
}
