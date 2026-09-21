#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <array>
#include <std_msgs/Float64.h>

ros::Publisher pub_steering_angle;  // 퍼블리셔 변수 선언

float pwm_servo = 0;
float steering_rad = 0;

const float pi{3.14}; // 파이값 상수로 선언
const float L{0.25}; // 차량 길이 선언
const float direction_straight = 1564;
const float pwm_resolution_servo = 200;

std::array<float, 2> Drive(std::array<float, 2> curr_pos, float curr_spd, float heading)
{
    curr_pos[0] += curr_spd * cos(heading);
    curr_pos[1] += curr_spd * sin(heading);
    return curr_pos;
}

class PurePursuit
{
public:
    std::array<float, 2> getLookaheadPoint(std::array<float, 2> curr_pos, const std::vector<std::array<float, 2>>& path, int lookahead_distance)
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

        int target_index = close_index + lookahead_distance;
        if (target_index >= path.size())
            target_index = path.size() - 1;

        std::array<float, 2> lookahead_point = path[target_index];
        return lookahead_point;
    }

    float getSteeringAngle(std::array<float, 2> curr_pos, std::array<float, 2> lookahead_point, float heading)
    {
        float Ld, alpha, steering_angle;

        Ld = getDist(lookahead_point, curr_pos);
        alpha = atan2((lookahead_point[1] - curr_pos[1]), (lookahead_point[0] - curr_pos[0])) - heading;
        steering_angle = atan2(2 * L * sin(alpha), Ld);

        // 라디안 각도로 변환
        float mapped_value = (steering_angle / (pi / 180)) / 30;
        //값출력
        std::cout << "Mapped Value: " << mapped_value << std::endl;

        // 각도를 다시 역으로 계산하여 pwm_servo에 할당
        float angle_deg = steering_angle * (180 / pi);
        pwm_servo = ((angle_deg * pwm_resolution_servo) / 30) + direction_straight;

        // pwm_servo 값의 범위 조정
        if (pwm_servo > 1763)
        {
        pwm_servo = 1763;
        }

        if (pwm_servo < 1363)
        {
        pwm_servo = 1363;
        }



        // steering_rad 값을 업데이트
        steering_rad = ((pwm_servo - 1563) / 200) * 30 * (pi / 180);

        return steering_angle;
    }

private:
    float getDist(std::array<float, 2> point1, std::array<float, 2> point2)
    {
        float dist = sqrt(pow(point2[0] - point1[0], 2) + pow(point2[1] - point1[1], 2));
        return dist;
    }
};

float curr_speed = 0.0;  // 현재 속도를 저장하는 변수

void velCallback(const std_msgs::Float64::ConstPtr& msg)
{
    curr_speed = msg->data;
}

void callbackFunction(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::vector<std::array<float, 2>> path;
    for (const auto& point : cloud.points)
    {
        std::array<float, 2> pos = {point.x, point.y};
        path.push_back(pos);
    }

    float heading{atan2(1, 0)}, steering_angle;
    int lookahead_distance{static_cast<int>(curr_speed * 2)};
    std::array<float, 2> curr_pos = {0, 0}, lookahead_point;
    PurePursuit pure_pursuit;

    while (ros::ok())
    {
        lookahead_distance = static_cast<int>(curr_speed * 2);
        lookahead_point = pure_pursuit.getLookaheadPoint(curr_pos, path, lookahead_distance);
        steering_angle = pure_pursuit.getSteeringAngle(curr_pos, lookahead_point, heading);
        curr_pos = Drive(curr_pos, curr_speed, heading + steering_angle);
        heading += steering_angle;

        // 원하는 동작을 수행하고 결과를 출력하거나 다른 작업을 수행할 수 있습니다.
        // 예를 들어, 현재 위치와 주행 경로, 스티어링 앵글 등을 출력할 수 있습니다.
        // 퍼블리시할 메시지 생성
        std_msgs::Float64 steering_angle_msg;
        steering_angle_msg.data = steering_angle;

    

        // 종료 조건을 설정하여 알고리즘을 종료할 수 있습니다.
        // 예를 들어, 목표 지점에 도달하면 알고리즘을 종료하도록 설정할 수 있습니다.
        // steering_angle를 /del 토픽으로 퍼블리시
        pub_steering_angle.publish(steering_angle_msg);

        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "pure_2");
    ros::NodeHandle nh;
    ros::Subscriber sub_vel = nh.subscribe<std_msgs::Float64>("vel", 10, velCallback);
    ros::Subscriber sub_lane = nh.subscribe<sensor_msgs::PointCloud2>("/lane", 10, callbackFunction);
    pub_steering_angle = nh.advertise<std_msgs::Float64>("/del", 10);
    
    ros::spin();
    
    return 0;
}
