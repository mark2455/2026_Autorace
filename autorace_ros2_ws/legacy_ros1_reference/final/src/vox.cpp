//1번

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


ros::Publisher pub;

void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud; // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    pcl::fromROSMsg(*msg, inputCloud);
    pcl::VoxelGrid<pcl::PointXYZ> vox;

    vox.setInputCloud (inputCloud.makeShared());
    vox.setLeafSize (0.06f, 0.06f, 0.06f); // set Grid Size(1.0m)
    vox.filter (filteredCloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filteredCloud, output);
    output.header.frame_id = "map";
    pub.publish(output);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "vox");
    ros::NodeHandle nh;
    // << Subscribe Topic >> //
    //topic name : /vLidarPC
    // topic type : sensor_msgs::PointCloud2
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/Laser2PointCloud", 1, callbackFcn);

    // << Publish Topic >>
    // topic name : /voxelPC
    // topic type : sensor_msgs::PointCloud2
    pub = nh.advertise<sensor_msgs::PointCloud2>("/vox", 1);
    ros::spin();
    return 0;
}
  // 여기에 PointCloud2 데이터를 처리하는 코드를 작성합니다.
  // 변환된 직교 좌표 데이터를 활용하여 클러스터링이나 다른 작업을 수행할 수 있습니다.


