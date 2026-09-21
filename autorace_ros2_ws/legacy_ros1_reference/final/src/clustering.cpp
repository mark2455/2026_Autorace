#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

ros::Publisher pub1;
ros::Publisher pub2;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<pcl::PointXYZ> inputCloud;

    pcl::fromROSMsg(*msg, inputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud (inputCloud.makeShared());

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.5); // set distance threshold = 1.5m
    ec.setMinClusterSize (10); // set Minimum Cluster Size
    ec.setMaxClusterSize (25000); // set Maximum Cluster Size
    ec.setSearchMethod (kdtree);
    ec.setInputCloud (inputCloud.makeShared());
    ec.extract (clusterIndices);

    int clusterN = 1;
    std::vector<pcl::PointIndices>::const_iterator it;
    for (it = clusterIndices.begin (); it != clusterIndices.end (); ++it){

        pcl::PointCloud<pcl::PointXYZ> filteredCloud;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        filteredCloud.push_back (inputCloud[*pit]);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(filteredCloud, output);
        output.header.frame_id = "map";

        clusterN == 1 ? pub1.publish(output) : pub2.publish(output);
        clusterN++;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;

    // << Subscribe Topic >>
    // topic name : /passPC
    // topic type : sensor_msgs::PointCloud2
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter", 1, callbackFcn);

    // << Publish Topic >>
    // topic name : /cluster1PC
    // topic type : sensor_msgs::PointCloud2
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/cluster1PC", 1);
    
    // << Publish Topic >>
    // topic name : /cluster2PC
    // topic type : sensor_msgs::PointCloud2
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("/cluster2PC", 1);
    ros::spin();
    return 0;
}
