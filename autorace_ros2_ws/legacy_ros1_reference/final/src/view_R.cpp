//1번
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void CallbackFunction(const sensor_msgs::PointCloud2::ConstPtr & msg )
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg,cloud);

  double x_mean, y_mean, num, den, a, b;
  int len = cloud.size();

  for(int i=0; i<len; i++)
  {
    x_mean += cloud.points[i].x;
    y_mean += cloud.points[i].y;
  }
  x_mean = x_mean/len;
  y_mean = y_mean/len;

  for(int i=0; i<len; i++)
  {
    num += (cloud.points[i].y - y_mean)*(cloud.points[i].x - x_mean);
    den += (cloud.points[i].x - x_mean)*(cloud.points[i].x - x_mean);
  }
  a = num/den;
  b = y_mean - a * x_mean;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p;
  for(int x=-2; x<3; x++)
  {
    p.x = x;
    p.y = a*x + b;
    p.z = 0;

    line_strip.points.push_back(p);
  }

  pub.publish(line_strip);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "view_R");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/filter_R",10,CallbackFunction);
  pub = nh.advertise<visualization_msgs::Marker>("/view_R",10);
  ros::spin();
  return 0;
}
