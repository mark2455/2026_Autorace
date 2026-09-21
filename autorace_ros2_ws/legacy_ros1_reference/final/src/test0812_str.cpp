#include <ros/ros.h>
#include <std_msgs/Float64.h>

class WebotControl {
public:
    WebotControl() : rate(10) {
        ros::NodeHandle nh;
        webot_ctrl_pub = nh.advertise<std_msgs::Float64>("/commands/servo/position", 1);
    }

    void run() {
        std_msgs::Float64 steer_msg;
        steer_msg.data = 0.5; // Adjust steering angle (0.5 is center, 0 is left, 1 is right)
        webot_ctrl_pub.publish(steer_msg);
        rate.sleep();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher webot_ctrl_pub;
    ros::Rate rate;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "webot_node");
    WebotControl webotControl;

    while (ros::ok()) {
        webotControl.run();
        ros::spinOnce();
    }

    return 0;
}