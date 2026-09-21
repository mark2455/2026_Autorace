#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

ros::Publisher velocity_pub;
ros::Publisher steering_pub;

// PID Controller parameters
double Kp = 1.0;    // Proportional gain (Default: 1.0)
double Ki = 0.1;    // Integral gain (Default: 0.1)
double Kd = 0.01;   // Derivative gain (Default: 0.01)


double error_sum = 0.0;   // Accumulated error for integral term
double prev_error = 0.0;  // Previous error for derivative term
double current_steering_angle = 0.0; 

void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    pcl::fromROSMsg(*msg, inputCloud);
    
  
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(inputCloud.makeShared());

    pcl::PointXYZ searchPoint;
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius =1.5; // set searching radius size = 3m
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (int i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            filteredCloud.points.push_back(inputCloud.points[pointIdxRadiusSearch[i]]);
        }
    }

    // Calculate distance from the origin (0, 0, 0)
    std::vector<float> distances;
    for (int i = 0; i < filteredCloud.points.size(); ++i) {
        float distance = std::sqrt(std::pow(filteredCloud.points[i].x, 2) + std::pow(filteredCloud.points[i].y, 2) +
                                   std::pow(filteredCloud.points[i].z, 2));
        distances.push_back(distance);
    }

    // Adjust velocity based on distance
    double max_distance = 3.0; // Maximum distance for full speed
    double min_distance = 0.1; // Minimum distance for stopped speed
    double max_speed = 3.0;    // Maximum speed
    double min_speed = 1.0;    // Minimum speed
    double max_steering_angle = 0.5; // Maximum steering angle (radians)
    double current_steering_angle = 0.0;

    std_msgs::Float64 velocity;
    std_msgs::Float64 steering_msg;
    for (int i = 0; i < distances.size(); ++i) {
        double distance = distances[i];

        // Calculate speed based on distance
        double speed = 1.2 + max_speed * (1.5*distance - min_distance) / (max_distance - min_distance);

        // Apply speed limits
        if (speed > max_speed) {
            speed = max_speed;
        } 
        else if (speed < min_speed) {
            speed = min_speed;
        }
        
        // Calculate steering angle based on distance and obstacle position
        double target_steering_angle = 0.0;
        if (filteredCloud.points[i].y < 0) {
            target_steering_angle = -max_steering_angle;  // Turn right for left-side obstacle
        } else if (filteredCloud.points[i].y > 0) {
            target_steering_angle = max_steering_angle; // Turn left
        } else {
            target_steering_angle = 0.0; 
        }
        
        // PID Control for steering angle
      
        double error = target_steering_angle - current_steering_angle;

        // Proportional term
        double p_term = Kp * error;

        // Integral term
        error_sum += error;
        double i_term = Ki * error_sum;

        // Derivative term
        double d_term = Kd * (error - prev_error);
        prev_error = error;

        double steering_angle = p_term + i_term + d_term;

        // Apply steering angle limits
        if (steering_angle > max_steering_angle) {
            steering_angle = max_steering_angle;
        } 
        else if (steering_angle < -max_steering_angle) {
            steering_angle = -max_steering_angle;
        }
	
	current_steering_angle = steering_angle;	
	
	if (distance >= 0.3){
	   steering_angle = 0.0;
	}
	
        velocity.data = speed;
        steering_msg.data = steering_angle;
        velocity_pub.publish(velocity);
        steering_pub.publish(steering_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter", 1, callbackFcn);
    velocity_pub = nh.advertise<std_msgs::Float64>("/vel", 1);
    steering_pub = nh.advertise<std_msgs::Float64>("/del", 1);
    
    ros::spin();

    return 0;
}
