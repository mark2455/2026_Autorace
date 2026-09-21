#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <array>
#include <vector>
#include <cmath>

const float pi{3.141592};
const float L{0.5};

std::vector<std::array<float, 2>> path;

float getDist(std::array<float, 2> point1, std::array<float, 2> point2)
{
    float dist = sqrt(pow(point2[0] - point1[0], 2) + pow(point2[1] - point1[1], 2));
    return dist;
}

std::array<float, 2> Drive(std::array<float, 2> curr_pos, int curr_spd, float heading)
{
    curr_pos[0] += curr_spd * cos(heading);
    curr_pos[1] += curr_spd * sin(heading);
    return curr_pos;
}

class PurePursuit
{
public:
    std::array<float, 2> getLookaheadPoint(std::array<float, 2> curr_pos, int lookahead_distance)
    {
        float min_dist{99999}, dist;
        int close_index;

        for (int i = 0; i < path.size(); i++)
        {
            dist = getDist(path[i], curr_pos);
            if (dist < min_dist)
            {
                min_dist = dist;
                close_index = i;
            }
        }

        int lookahead_index = std::min(close_index + lookahead_distance, static_cast<int>(path.size()) - 1);
        std::array<float, 2> lookahead_point = path[lookahead_index];
        return lookahead_point;
    }

    float getSteeringAngle(std::array<float, 2> curr_pos, std::array<float, 2> lookahead_point, float heading)
    {
        float Ld, alpha, steering_angle;

        Ld = getDist(lookahead_point, curr_pos);
        alpha = atan2((lookahead_point[1] - curr_pos[1]), (lookahead_point[0] - curr_pos[0])) - heading;
        steering_angle = atan2(2 * L * sin(alpha), Ld);
        return steering_angle;
    }
};

void pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    path.clear();
    for (auto &pose : msg->poses)
    {
        std::array<float, 2> point = {static_cast<float>(pose.pose.position.x), static_cast<float>(pose.pose.position.y)};
        path.push_back(point);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit0812");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/steer_msg", 10);
    ros::Subscriber path_sub = nh.subscribe("/path_topic", 10, pathCallback);

    geometry_msgs::Twist cmd_vel_msg;

    float heading = atan2(1, 0);
    int curr_spd = 1;
    int lookahead_distance = curr_spd * 2;
    std::array<float, 2> curr_pos = {0, 0}, lookahead_point;
    PurePursuit pure_pursuit;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (!path.empty())
        {
            lookahead_point = pure_pursuit.getLookaheadPoint(curr_pos, lookahead_distance);
            float steering_angle = pure_pursuit.getSteeringAngle(curr_pos, lookahead_point, heading);
            heading += curr_spd * tan(steering_angle) / L;
            curr_pos = Drive(curr_pos, curr_spd, heading);

            cmd_vel_msg.linear.x = curr_spd;
            cmd_vel_msg.angular.z = steering_angle;

            cmd_vel_pub.publish(cmd_vel_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
