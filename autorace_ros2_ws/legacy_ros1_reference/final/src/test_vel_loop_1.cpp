#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

ros::Publisher velocity_pub;
bool is_kdtree_initialized = false;
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());

void callbackLeft(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    pcl::fromROSMsg(*msg, inputCloud);

    if (inputCloud.size() == 0) {
        // No data found, handle this case (e.g., set default values)
        std_msgs::Float64 velocity;
        velocity.data = 1.33; // Set default speed

        velocity_pub.publish(velocity);
        return; // Exit the callback function
    }

    if (!is_kdtree_initialized) {
        kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        kdtree->setInputCloud(inputCloud.makeShared());
        is_kdtree_initialized = true;
    }

    pcl::PointXYZ searchPoint;
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 1.4; // set searching radius size = 3m
    kdtree->radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i) {
        filteredCloud.points.push_back(inputCloud.points[pointIdxRadiusSearch[i]]);
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
    double min_speed = 1.3;    // Minimum speed

    std_msgs::Float64 velocity;
    if (min_distance_index != -1) {
        double distance = distances[min_distance_index];

        // Calculate speed based on distance
        double speed = 1.33 + max_speed * (distance - min_distance_threshold) / (max_distance - min_distance_threshold);

        // Apply speed limits
        if (speed > max_speed) {
            speed = max_speed;
        } else if (speed < min_speed) {
            speed = min_speed;
        }

        velocity.data = speed;
    } else {
        velocity.data = min_speed;
    }
    velocity_pub.publish(velocity);
}

void callbackRight(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Similar implementation as callbackLeft
    // Modify as per your requirements for the right filter data
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    pcl::fromROSMsg(*msg, inputCloud);

    if (inputCloud.size() == 0) {
        // No data found, handle this case (e.g., set default values)
        std_msgs::Float64 velocity;
        velocity.data = 1.33; // Set default speed

        velocity_pub.publish(velocity);
        return; // Exit the callback function
    }

    if (!is_kdtree_initialized) {
        kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        kdtree->setInputCloud(inputCloud.makeShared());
        is_kdtree_initialized = true;
    }

    pcl::PointXYZ searchPoint;
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 1.4; // set searching radius size = 3m
    kdtree->radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i) {
        filteredCloud.points.push_back(inputCloud.points[pointIdxRadiusSearch[i]]);
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
    double min_speed = 1.3;    // Minimum speed

    std_msgs::Float64 velocity;
    if (min_distance_index != -1) {
        double distance = distances[min_distance_index];

        // Calculate speed based on distance
        double speed = 1.33 + max_speed * (distance - min_distance_threshold) / (max_distance - min_distance_threshold);

        // Apply speed limits
        if (speed > max_speed) {
            speed = max_speed;
        } else if (speed < min_speed) {
            speed = min_speed;
        }

        velocity.data = speed;
    } else {
        velocity.data = min_speed;
    }
    velocity_pub.publish(velocity);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "test_vel_1");
    ros::NodeHandle nh;

    ros::Subscriber sub_L = nh.subscribe<sensor_msgs::PointCloud2>("/filter_L", 1, callbackLeft);
    ros::Subscriber sub_R = nh.subscribe<sensor_msgs::PointCloud2>("/filter_R", 1, callbackRight);
    velocity_pub = nh.advertise<std_msgs::Float64>("/vel", 1);

    ros::Rate loop_rate(10); // Set loop rate to 10 Hz

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
