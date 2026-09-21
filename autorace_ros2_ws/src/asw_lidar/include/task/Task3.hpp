#include "lidar_lib/lidar_lib.h"
#include <coss_msgs/msg/coss.hpp>
#define MAX_SIZE 1

//---------------박스 정적 회피------------------|
//-------ROI: 내 차선, 내 전방 15m----------------------|
class Task3 : public Lidar_Func{
private:
// param
    float voxel_size;
    float roi[4];
    float param[3];
    vector<PointType> center;
// rotary
    bool rotary_flag = false;
    bool stop_flag = false;
    int is_right = -1; // 0이면 left, 1이면 right
    int obj_count = 0;
    bool change_state = false;
    vector<double> pointX;
    vector<double> pointY;

// pcl
    pcl::PointCloud<PointType>::Ptr msg_;
    sensor_msgs::PointCloud2::SharedPtr msg;
// pub, sub
//--------------Publisher------------|
    ros::Publisher pub_points_;
    ros::Publisher pub_center_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_status_flag_;
    ros::Publisher pub_coss_state_;
//-------------Subscriber------------|
    ros::Subscriber sub_points;
    asw_lidar::lidar_setting flag_msg;
    coss_msgs::msg::Coss coss_msg;
public:
    Task3(){
        initsetup();
        msg.reset(new sensor_msgs::PointCloud2());
        msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    ~Task3()
    {
        ROS_INFO("Task 3 Clear");
    }
    void initsetup();
    void run();
    void scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan);
    void vector_clear();
    void Rotary();
    int Rotary_Decision(vector<double> vy);
};
void Task3::scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan) {
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, *msg);
    pcl::fromROSMsg(*msg, *msg_);
}
void Task3::initsetup(){
    ros::param::get("/Task3/min_x_",  roi[0]);
    ros::param::get("/Task3/max_x_",  roi[1]);
    ros::param::get("/Task3/min_y_",  roi[2]);
    ros::param::get("/Task3/max_y_",  roi[3]);
    ros::param::get("/Task3/tolerance",  param[0]);
    ros::param::get("/Task3/cluster_size_min",  param[1]);
    ros::param::get("/Task3/cluster_size_max",  param[2]);
    ros::param::get("/Task3/voxel_size", voxel_size);
    //------------pub----------------------------------------------------------------|
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("passed_points", 10);
    pub_center_ = nh_.advertise<visualization_msgs::Marker>("center_points", 10);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("wayPoint", 10);
    pub_status_flag_ = nh_.advertise<asw_lidar::lidar_setting>("/static_flag_topic", 1);
    pub_coss_state_ = nh_.advertise<coss_msgs::msg::Coss>("coss_state", 1);
    //------------sub----------------------------------------------------------------|
    sub_points = nh_.subscribe("/scan", 1, &Task3::scanCallback, this);
}
void Task3::vector_clear(){
    center.clear();
}
// //로터리
// void Task3::Rotary() {
//     if(center.size() == 0 && is_right == -1){
//         stop_flag = false;
//         pointY.clear();
//     }
//     else if(!rotary_flag && center.size() != 0 && is_right == -1) {
//         stop_flag = true;
//         rotary_decision_flag = true;
//         cout << "STOP For Decision" << endl;
//     }
//     if(rotary_decision_flag && center.size() != 0) {
//         if(is_right == -1){
//             pointY.push_back(center[0].y);
//             cout << pointY.size() << endl;
//             if(pointY.size() == MAX_SIZE) {
//                 is_right = 1;
//                 cout << is_right << endl;
//             }
//         }
//         cout << "Ready to Start" << endl;
//         if((is_right != -1 && center[0].x > 0.9) || center.size() == 0){
//             stop_flag = false;
//             rotary_flag = true;
//             rotary_decision_flag = false;
//             cout << "!!!!!!!!! GO !!!!!!!!!" << endl;
//         }
//     }
//     // After Rotary
//     if(rotary_flag){
//         if(center.size() != 0 && (center[0].x >= 0.13 && center[0].x < 0.5) && fabs(center[0].y) <= 0.1) {
//             stop_flag = true;
//             cout << "차단기 STOP" << endl;
//         }
//         else{
//             stop_flag = false;
//         }
//     }
//     coss_msg.lidar_stop_flag = stop_flag;
//     coss_msg.is_right = is_right;
//     cout << is_right << endl;
//     pub_coss_state_.publish(coss_msg);
// }

int Task3::Rotary_Decision(vector<double> vy){
    double ydiff = vy.front() - vy.back();
    cout << "ydiff : " << ydiff << endl;
    if(ydiff >= 0) return 1;
    else return 0;
}

void Task3::Rotary(){
    if(center.size() == 0){
        stop_flag = false;
        if(obj_count >= 45) {
            change_state = true;
            cout << "state 3 on" << endl;
        }
        obj_count = 0;
    }
    else{
        stop_flag = true;
        obj_count++;
        cout << obj_count << endl;
    }
    coss_msg.lidar_stop_flag = stop_flag;
    coss_msg.state3 = change_state;
    pub_coss_state_.publish(coss_msg);
}

// int Task3::Rotary_Decision(vector<double> vx, vector<double> vy){
//     double xdiff = .0; // 양수면 가까워짐, 음수면 멀어짐
//     double ydiff = .0; // 양수면 오른쪽으로, 음수면 왼쪽으로 이동

//     // x,y 최댓값 최솟값 인덱스
//     int x_min_idx = distance(vx.begin(), min_element(vx.begin(), vx.end()));
//     int x_max_idx = distance(vx.begin(), max_element(vx.begin(), vx.end()));
//     int y_min_idx = distance(vy.begin(), min_element(vy.begin(), vy.end()));
//     int y_max_idx = distance(vy.begin(), max_element(vy.begin(), vy.end()));

//     cout << "x_min_index : " << x_min_idx << ", " << "x_max_index : " << x_max_idx << endl;
//     cout << "y_min_index : " << y_min_idx << ", " << "y_max_index : " << y_max_idx << endl;

//     // xdiff 산출
//     if((x_min_idx == 0 && x_max_idx == 29) || (x_min_idx == 29 && x_max_idx == 0)) {
//         xdiff = vx.front() - vx.back();
//     }
//     else if(x_min_idx != 0 || x_min_idx != 29){
//         xdiff = vx.front() - vx[x_min_idx]; // xdiff > 0 가까워지는 것으로 판단
//         cout << "xdiff : " << vx.front() << ", " << vx[x_min_idx] << endl;
//         cout << "가까워짐" << endl;
//     }
//     else if(x_max_idx != 0 || x_max_idx != 29){
//         xdiff = vx.front() - vx[x_max_idx]; // xdiff < 0 멀어지는 것으로 판단
//         cout << "xdiff : " << vx.front() << ", " << vx[x_max_idx] << endl;
//         cout << "멀어짐" << endl;
//     }

//     // ydiff 산출
//     if((y_min_idx == 0 && y_max_idx == 29) || (y_min_idx == 29 && y_max_idx == 0)){
//         ydiff = vy.front() - vy.back();
//     }
//     else if(y_min_idx != 0 || y_min_idx != 29) {
//         ydiff = vy.front() - vy[y_min_idx]; // ydiff > 0 좌에서 우로 이동
//         cout << "ydiff : " << vy.front() << ", " << vy[y_min_idx] << endl;
//         cout << "좌에서 우로" << endl;
//     }
//     else if(y_max_idx != 0 || y_max_idx != 29) {
//         ydiff = vy.front() - vy[y_max_idx]; // ydiff < 0 우에서 좌로 이동
//         cout << "ydiff : " << vy.front() << ", " << vy[y_max_idx] << endl;
//         cout << "우에서 좌로" << endl;
//     }

//     cout << "##### Rotary Decision #####" << endl;
//     cout << "xdiff : " << xdiff << ", " << "ydiff : " << ydiff << endl;

//     if(xdiff < 0 && ydiff < 0) return 0; // 차는 시계 방향으로 움직임
//     else if(xdiff < 0 && ydiff >= 0) return 1; // 차는 반시계 방향으로 움직임
//     else if(xdiff >= 0 && ydiff < 0) return 1; // 차는 반시계 방향으로 움직임
//     else if(xdiff >= 0 && ydiff >= 0) return 0; // 차는 시계 방향으로 움직임
// }
void Task3::run(){
    vector_clear();
    set_roi(msg_, roi);
    voxel(msg_, voxel_size);
    clustering(msg_,center, param);
    Rotary();
    //visualize_center(center);
}