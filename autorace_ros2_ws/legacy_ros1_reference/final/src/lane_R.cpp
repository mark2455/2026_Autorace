#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

ros::Publisher pub;

void callbackFunction(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  double x_mean = 0.0, y_mean = 0.0, num = 0.0, den = 0.0, a = 0.0, b = 0.0;
  int len = cloud.size();

  for(int i = 0; i < len; i++)
  {
    x_mean += cloud.points[i].x;
    y_mean += cloud.points[i].y;
  }
  x_mean = x_mean / len;
  y_mean = y_mean / len;

  for(int i = 0; i < len; i++)
  {
    num += (cloud.points[i].y - y_mean) * (cloud.points[i].x - x_mean);
    den += (cloud.points[i].x - x_mean) * (cloud.points[i].x - x_mean);
  }
  a = num / den;
  b = y_mean - a * x_mean;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "map";

  pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_R");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter_R", 10, callbackFunction);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/lane_R", 10);
  ros::spin();
  return 0;
}
