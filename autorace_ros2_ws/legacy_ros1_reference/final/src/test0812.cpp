#include <ros/ros.h>
#include <std_msgs/Float64.h>

class WebotControl {
public:
    WebotControl() : rate(10) {
        ros::NodeHandle nh;
        webot_ctrl_pub = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 1);
    }

    void run() {
        std_msgs::Float64 speed_msg;
        speed_msg.data = 1000; // 모터 속도 설정 (0은 정지, 1000은 보통 속도)
        webot_ctrl_pub.publish(speed_msg);
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