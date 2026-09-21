#include "lidar_lib/lidar_lib.h"

//---------------처음 아크릴 회피------------------|
class Task6 : public Lidar_Func{
private:
// param
    float voxel_size;
    float roi[4];
    float param[3];
    vector<PointType> center_;
// pcl
    pcl::PointCloud<PointType>::Ptr msg_;
// pub, sub
//--------------Publisher------------|
    ros::Publisher pub_points_;
    ros::Publisher pub_center_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_status_flag_;
//-------------Subscriber------------|
    ros::Subscriber sub_points;
    sensor_msgs::PointCloud2::SharedPtr msg;
    asw_lidar::lidar_setting flag_msg;
public:
    Task6(){
        initsetup();
        msg.reset(new sensor_msgs::PointCloud2());
        msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    ~Task6()
    {
        ROS_INFO("Task 6 Clear");
    }
    void initsetup();
    void run();
    void scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan);
    void vector_clear();
    void decision();
};
void Task6::scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan) {
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, *msg);
    pcl::fromROSMsg(*msg, *msg_);
}
void Task6::initsetup(){
    ros::param::get("/Task6/min_x_",  roi[0]);
    ros::param::get("/Task6/max_x_",  roi[1]);
    ros::param::get("/Task6/min_y_",  roi[2]);
    ros::param::get("/Task6/max_y_",  roi[3]);
    ros::param::get("/Task6/tolerance",  param[0]);
    ros::param::get("/Task6/cluster_size_min",  param[1]);
    ros::param::get("/Task6/cluster_size_max",  param[2]);
    ros::param::get("/Task6/voxel_size", voxel_size);
    //------------pub----------------------------------------------------------------|
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("passed_points", 10);
    pub_center_ = nh_.advertise<visualization_msgs::Marker>("center_points", 10);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("wayPoint", 10);
    pub_status_flag_ = nh_.advertise<asw_lidar::lidar_setting>("/static_flag_topic", 1);
    //------------sub----------------------------------------------------------------|
    sub_points = nh_.subscribe("/scan", 1, &Task6::scanCallback, this);
}
void Task6::vector_clear(){
    center_.clear();
}
void Task6::decision(){
    //안전거리 전방 10m(morai 7m) LiDAR센서 거리 전방 18m(morai 15m)
    float moving_distance = 8.0;
    cout << "count: " << center_.size() << endl;
    //----------------센터점이 존재하는 경우---------------------|
    if(!center_.empty()){
        float obstacle_distance = center_[0].z;
        cout << "----------장애물 인식--------------" << endl;
        //-------------장애물거리와 안전거리가 같을 때------------|
        if(obstacle_distance <= moving_distance){
            cout << "---------안전거리 안에 장애물---------------" << endl;
            cout << "---------회 피 시 작--------------------" << endl;
            flag_msg.flag = true;
            pub_status_flag_.publish(flag_msg);
        }
    }
    //--------------센터점이 존재하지 않을 때--------------|
    else{
        cout << "GOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
        flag_msg.flag = false;
        pub_status_flag_.publish(flag_msg);
    }
}
void Task6::run(){
    vector_clear();
    set_roi(msg_, roi);
    voxel(msg_, voxel_size);
    clustering(msg_,center_, param);
    decision();
    //visualize_center(center_);
}