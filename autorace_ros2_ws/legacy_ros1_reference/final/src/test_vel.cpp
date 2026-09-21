#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

ros::Publisher velocity_pub;

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

    float radius = 1.5; // set searching radius size = 3m
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

    // Find minimum distance
    float min_distance = std::numeric_limits<float>::max();
    int min_distance_index = -1;
    for (int i = 0; i < distances.size(); i++) {
        if (distances[i] < min_distance) {
            min_distance = distances[i];
            min_distance_index = i;
        }
    }

    // Adjust velocity based on distance
    double max_distance = 5.0; // Maximum distance for full speed
    double min_distance_threshold = 0.1; // Minimum distance for stopped speed
    double max_speed = 2.0;    // Maximum speed
    double min_speed = 1.0;    // Minimum speed

    std_msgs::Float64 velocity;
    if (min_distance_index != -1) {
        double distance = distances[min_distance_index];

        // Calculate speed based on distance
        double speed = 1.3 + max_speed * (distance - min_distance_threshold) / (max_distance - min_distance_threshold);

        // Apply speed limits
        if (speed > max_speed) {
            speed = max_speed;
        } 
        else if (speed < min_speed) {
            speed = min_speed;
        }

        velocity.data = speed;
    }
    else {
        velocity.data = min_speed;
    }
    velocity_pub.publish(velocity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_vel");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter", 1, callbackFcn);
    velocity_pub = nh.advertise<std_msgs::Float64>("/vel", 1);

    ros::Rate loop_rate(10); // Set loop rate to 10 Hz

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
