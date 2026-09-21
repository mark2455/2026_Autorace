#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "speednode");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 1);
    std_msgs::Float64 speed_msg;

    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/commands/servo/position", 1);
    std_msgs::Float64 steer_msg;

    while (ros::ok()) {
        speed_msg.data = 1000;
        pub.publish(speed_msg);

        steer_msg.data = 0.5;
        pub2.publish(steer_msg);
    }

    return 0;
}