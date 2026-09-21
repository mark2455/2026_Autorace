//평균값 시각화 3번째

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

ros::Publisher pub_mean;  // 평균값 퍼블리셔 변수 선언
ros::Publisher pub_marker;  // 마커 퍼블리셔 변수 선언

void callbackFunction(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    double x_mean = 0.0, y_mean = 0.0;
    int len = cloud.size();

    for (int i = 0; i < len; i++)
    {
        x_mean += cloud.points[i].x;
        y_mean += cloud.points[i].y;
    }
    x_mean = x_mean / len;
    y_mean = y_mean / len;

    // 평균값을 퍼블리시할 메시지 생성
    sensor_msgs::PointCloud2 mean_msg;
    pcl::PointCloud<pcl::PointXYZ> mean_cloud;
    mean_cloud.push_back(pcl::PointXYZ(x_mean, y_mean, 0.0));
    pcl::toROSMsg(mean_cloud, mean_msg);
    mean_msg.header.frame_id = "map";

    // 평균값 시각화를 위한 Marker 메시지 생성
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "mean_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;  // 포인트 크기
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;  // 투명도
    marker.color.r = 1.0;  // 색상 (빨강)
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // x, y 평균 데이터 포인트를 생성하여 마커에 추가
    int num_points = 10;  // 포인트 개수
    double spacing = 0.5;  // 포인트 간격
    double start_x = x_mean - (num_points / 2.0) * spacing;
    double start_y = y_mean - (num_points / 2.0) * spacing;
    for (int i = 0; i < num_points; i++)
    {
        double x = start_x + i * spacing;
        double y = start_y + i * spacing;
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        marker.points.push_back(point);
    }

    // 평균값을 퍼블리시 및 시각화
    pub_mean.publish(mean_msg);
    ros::Duration(0.1).sleep();  // 시간 지연을 통해 충돌 방지

    // 시각화된 포인트를 퍼블리시
    pub_marker.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_mean");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter_mean", 10, callbackFunction);
    pub_mean = nh.advertise<sensor_msgs::PointCloud2>("/mean", 10);
    pub_marker = nh.advertise<visualization_msgs::Marker>("mean_marker", 10);

    // 데이터를 받지 못해도 종료되지 않도록 루프 생성
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
