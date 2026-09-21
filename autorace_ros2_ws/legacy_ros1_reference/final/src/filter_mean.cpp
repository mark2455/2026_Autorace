//lane_view 파일 시각화 도와주는 필터파일입니다.

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>


ros::Publisher pub;
void callbackFcn(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> inputCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud_x;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud_y;

    pcl::fromROSMsg(*msg, inputCloud);

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pcl::PassThrough<pcl::PointXYZ> pass_y;


    // X 기준 필터링 
    pass_x.setInputCloud (inputCloud.makeShared());
    pass_x.setFilterFieldName ("x");
    // set Axis(x)
    pass_x.setFilterLimits (-2.0, 0.0);
    // x : -1.0 ~ 1.0
    pass_x.setFilterLimitsNegative (false); //설정영역 true 또는 이외의 부분 필터링 false
    pass_x.filter (filteredCloud_x);


    // Y 기준 필터링
    pass_y.setInputCloud (filteredCloud_x.makeShared());
    pass_y.setFilterFieldName ("y");
    // set Axis(x)
    pass_y.setFilterLimits (-0.5, 0.5);
    // y : -1.0 ~ 1.0
    pass_y.setFilterLimitsNegative (false); //설정영역 true 또는 이외의 부분 필터링 false
    pass_y.filter (filteredCloud_y);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filteredCloud_y, output);
    output.header.frame_id = "map";
    pub.publish(output);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "filter_mean");
    ros::NodeHandle nh;
    // << Subscribe Topic >>
    // topic name : /voxelPC
    // topic type : sensor_msgs::PointCloud2
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/vox", 1, callbackFcn);

    // << Publish Topic >>
    // topic name : /passPC
    // topic type : sensor_msgs::PointCloud2
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filter_mean", 1);
    ros::spin();
    return 0;
}
  // 여기에 PointCloud2 데이터를 처리하는 코드를 작성합니다.
  // 변환된 직교 좌표 데이터를 활용하여 클러스터링이나 다른 작업을 수행할 수 있습니다.


