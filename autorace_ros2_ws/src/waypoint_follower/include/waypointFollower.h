#ifndef WAYPOINTFOLLOWER_Hcp
#define WAYPOINTFOLLOWER_H

#include <coss_msgs/msg/coss.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <waypoint_maker/msg/lane.hpp>
#include <waypoint_maker/msg/waypoint.hpp>
#include <waypoint_maker/msg/state.hpp>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Float32.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>
#include <numeric>

#include <sensor_msgs/Imu.h>

#include <tf/tf.h>
#include "ros/time.h"





#define INTER_TIME_PLUS 1000000000
#define INTER_TIME_MIN 90000000
#define INTER_TIME_MAX 200000000
#define INTER_SPEED_TIME_MAX 3600000000


using namespace std;

class WaypointFollower 
{
private:

	double j_speed = 0;// for Jeong Min
	double j_steer = 0;// for Jeong Min

	bool static_flag = false;// for Jeong Min

	const double WHEEL_BASE = 1.0;
	const double MAX_SEARCH_DIST = 5.0;
	const double MAX_SUM = 58.0;
	const double MIN_SUM = -150.0;

	const double KI = 0.09; //0.09
	const double PID_FIT = 1.0;

	const double STRAGHT_LD = 5.0;
	const double _courseVE_LD = 2.5;
	const double BIG_CURVE_LD = 3.0;
	const double PARKING_LD = 2.0;

	int waypoints_size_;
	int target_index_;
	
	vector<waypoint_maker::msg::Waypoint> waypoints_;
	
	double dist_;

	int current_mission_state_;
	int loader_number_;

	int waypoint_min_;
	int closest_waypoint_;
	int coss_state;
	int spd_state_;
	int per_count;
	bool is_pose_;
	bool is_course_;
	bool is_lane_;
	bool is_control_;
	bool is_obs_detect_;
	bool vision_check_;
	double is_alpha ; 
	geometry_msgs::PoseStamped cur_pose_;
	geometry_msgs::Twist cur_vel_;
	double lookahead_dist_;
	double cur_course_;
	double cur_speed_; //speed over ground -> nmea sentence GPRMC 7

	int real_mission_state_;
	
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	
	ros::Publisher control_pub_;
	ros::Publisher lane_number_pub_;
	ros::Publisher state_pub_;
	ros::Publisher auto_pub_;



	ros::Subscriber odom_sub_;
	ros::Subscriber lane_sub_;
	ros::Subscriber vision_sub_;
	ros::Subscriber state_sub_;
	ros::Subscriber imu_sub_;

	ros::Subscriber line_sub_;
	ros::Subscriber left_line_sub_;
	ros::Subscriber right_line_sub_;

	ros::Subscriber vertical_sub_;
	ros::Subscriber obst_sub_;

	ros::Subscriber static_sub_;
	ros::Subscriber static_steer_;
	ros::Subscriber track_steer_sub_;
	ros::Subscriber auto_sub_;
	ros::Subscriber stopline_sub_;
	ros::Subscriber redzone_sub_;
	ros::Subscriber grayzone_sub_;
	ros::Subscriber servo_sub_; //커브 돌 때 서보 각도
	
	
	ackermann_msgs::AckermannDriveStamped control_msg_;
	waypoint_maker::msg::State lane_number_msg_;
	coss_msgs::msg::Coss coss_msg_;
	vector<geometry_msgs::PoseStamped> traffic_stop_;
	std_msgs::Bool is_auto_msg_;
	ros::ServiceClient service_client_ ;//service
	
	
	//동적
	bool is_obs_detect_dy_;
	
	//굴절
	bool is_line_vertical_;
	//오르막
	bool is_hill_stop_done_;
	bool start_hill_stop_;

	bool is_first_stop_done_;
	bool start_first_stop_;
	bool nogps_steering_;
	
	bool is_third_stop_done_;
	bool start_third_stop_;
	bool nogps_steering_2_;

	//time fit
	double start_sec_;
	double during_sec_;
	bool gear_flag;
	double n_gps_start_sec_;
	double n_gps_during_sec_;
	double n_gps_start_sec2_;

	//service
    bool do_service_once_;
	
	double ex_x_, ex_y_;
    unsigned int ex_time_;
    int inter_time_;
    
    double sum_error_;
    double accel_, brake_;

	int parking_count_;
	bool is_backward_;
    
    bool is_dynamic_finished_;
	bool dynamic_check_flag_;
	double speed_;
	double cur_steer;
    double camera_angle_;
	double left_camera_angle_;
	double right_camera_angle_;
	double lidar_angle_;

	bool is_vertical_;

	double parking_dist_;
	float track_steer_;
	double filtered_yaw ;
	double orientation_z ;

	bool state[6];
	double lidar_steer_;
	int auto_flag;
	bool stoponce_flag;
	bool is_auto_;
	bool stopline_;
	bool redzone_;
	bool grayzone_;
	bool lidar_stop_flag_;
	int is_right_;
	double parking_steer_;
	std::string path_tracking;

	bool is_rear;
	bool is_front;
	bool is_stop;
	int rear_count;
	int front_count;
	int go_count; 
	int is_rear_count;
	double servo_position_;
	bool curve_state;
	int right_cam_count;
	int cam_count;

	public:
	WaypointFollower() 
	{
		initSetup();
	}

	~WaypointFollower() 
	{
		waypoints_.clear();
	}
	


	void initSetup() 
	{
	    ros::Time::init();
		control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("follower_cmd", 10);

		lane_number_pub_ = nh_.advertise<waypoint_maker::msg::State>("lane_number_msg_",1);
		odom_sub_ = nh_.subscribe("odometry/filtered", 1, &WaypointFollower::OdomCallback, this);
		state_pub_ = nh_.advertise<coss_msgs::msg::Coss>("mission_state",1);

		lane_sub_ = nh_.subscribe("final_waypoints", 1, &WaypointFollower::LaneCallback, this);
		state_sub_ = nh_.subscribe("coss_state",1,&WaypointFollower::StateCallback,this);
		vision_sub_ = nh_.subscribe("/vision_check", 10, &WaypointFollower::TrafficSignCallback,this);
		imu_sub_ = nh_.subscribe("/imu",1,&WaypointFollower::ImuCallback,this);
		
		// coss_topic_sub
		auto_sub_ = nh_.subscribe("/dev/null",1,&WaypointFollower::AutoCallback,this);
		auto_pub_ = nh_.advertise<std_msgs::Bool>("auto_flag", 10);
		
		line_sub_ = nh_.subscribe("/lane_detector_jeju_2/camera_ackermann",1,&WaypointFollower::LineCallback,this);
		left_line_sub_ = nh_.subscribe("/kuurack_left",1,&WaypointFollower::LeftLineCallback,this);
		right_line_sub_ = nh_.subscribe("/kuurack_right",1,&WaypointFollower::RightLineCallback,this);
		stopline_sub_ = nh_.subscribe("/stopline_detected",1,&WaypointFollower::StoplineCallback,this);
		redzone_sub_ = nh_.subscribe("/redzone_flag",1,&WaypointFollower::RedzoneCallback,this);
		grayzone_sub_ = nh_.subscribe("/gray_detected",1,&WaypointFollower::GrayCallback,this);
		servo_sub_ = nh_.subscribe("/commands/servo/position",1,&WaypointFollower::ServoCallback,this);
// for Jeong Min
		// static_sub_ = nh_.subscribe("/static_flag_topic",1,&WaypointFollower::StaticFlagCallback,this);
		// static_steer_ = nh_.subscribe("/static_steer_topic",1,&WaypointFollower::StaticSteerCallback,this);
// for Jeong Min
		track_steer_sub_ = nh_.subscribe("/Car_Control/SteerAngle_Int32",1,&WaypointFollower::TrackSteerCallback,this);

		// vertical_sub_ = nh_.subscribe("/dynamic_stop/lidar_ackermann",1,&WaypointFollower::VerticalCallback,this);
		//vertical_sub_ = nh_.subscribe("/ctrl_gps",1,&WaypointFollower::VerticalCallback,this);
		
			
		waypoints_size_ = 0;

		dist_ = 100.0;
		lookahead_dist_ = 0.8;
		current_mission_state_ = -1;
		waypoint_min_ = -1;
		parking_count_ = -3;
		per_count = 0;
		is_pose_ = false;
		is_course_ = false;
		is_lane_ = false;
		is_control_ = false;

		is_obs_detect_dy_ = false;	//동적
		target_index_ = 0;
		//오르막
		is_hill_stop_done_ = false;
		start_hill_stop_ = false;

		//음영구역
		is_first_stop_done_ = false;
		start_first_stop_ = false;
		nogps_steering_ = false;
	
		is_third_stop_done_ = false;
		start_third_stop_ = false;
		nogps_steering_2_ = false;

		gear_flag = false;
		is_vertical_ = false;
		is_line_vertical_ = false;

		is_auto_ = false;
		vision_check_ = true;
		stoponce_flag = false;
		auto_flag = 0;
		spd_state_ = 0;

		is_obs_detect_ = false;
		
		//service
   		do_service_once_ = false;
    	inter_time_ = 0;

		cur_course_ = .0;
		cur_speed_ = .0;
		loader_number_ = 0;
		ex_x_ = 0;
		ex_y_ = 0;
		ex_time_ = 0;
		sum_error_ = 0;
		n_gps_start_sec_ = .0;
		n_gps_during_sec_ = .0;
		n_gps_start_sec2_ = .0;

		is_backward_ = false;
		dynamic_check_flag_ = false;
		
		//COSS 변수
		speed_ = .0;
		cur_steer = .0;
    	camera_angle_ = .0;
		left_camera_angle_ = .0;
		right_camera_angle_ = .0;
		lidar_angle_ = .0;
		track_steer_ = .0;
		accel_ = 0;
		brake_ = 0;
		is_alpha = .0;
		filtered_yaw = .0;
		orientation_z = .0 ; 
		coss_state = 1;
		lidar_steer_ = .0;
		stopline_ = false;
		redzone_ = false;
		grayzone_ = false;
		lidar_stop_flag_ = false;
		is_right_ = -1;
		// for parking 
		is_rear =false;
		is_front=false;
		is_stop = false;
		curve_state = false;
		servo_position_ = 0.546;

		rear_count=0;
		front_count=0;
		parking_steer_ = .0;
		go_count = 0 ; 
		right_cam_count = 0;
		is_rear_count = 0;
		cam_count = 0;
		for (bool & value : state) value = false;

		real_mission_state_ = 0;
        nh_.getParam("/waypoint_follower_node/is_dynamic_finished", is_dynamic_finished_);
        nh_.getParam("/waypoint_follower_node/is_parking_dist", parking_dist_);
	}

	double calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) 
	{
		return sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y - pose2.pose.position.y, 2));
	}

	void TrackSteerCallback(const std_msgs::Float32::ConstSharedPtr& msg)
	{
		track_steer_= msg->data;
	}

// // for Jeong Min
// 	void StaticFlagCallback(const std_msgs::Bool::ConstSharedPtr& msg){
// 		static_flag = msg->data;
// 	}

// 	void StaticSteerCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr& acker_msg){
// 		j_speed = acker_msg->drive.speed;
// 		j_steer = acker_msg->drive.steering_angle;
// 	}
// for Jeong Min

	void ServoCallback(const std_msgs::Float64::ConstSharedPtr &steer_msg)
	{
		servo_position_ = steer_msg->data;
		if (!curve_state)
		{
			if (servo_position_ < 0.36 && coss_state == 1)
			{
				//state[1] = true;
			}
		}
		curve_state = true;
	}

	void RedzoneCallback(const coss_msgs::msg::Coss::ConstSharedPtr &redzone_msg)
	{
		//state[0] = redzone_msg->state1;
		redzone_ = redzone_msg->redzone_flag;
	}

	void StoplineCallback(const coss_msgs::msg::Coss::ConstSharedPtr &coss_msg)
	{
		
		//state[1] = coss_msg->state2;
		stopline_ = coss_msg->state2;		
	}
	
	void GrayCallback(const coss_msgs::msg::Coss::ConstSharedPtr &coss_msg)
	{
		grayzone_ = coss_msg->gray_flag;
	}

	void OdomCallback(const nav_msgs::Odometry::ConstSharedPtr &odom_msg) 
	{
		cur_pose_.header = odom_msg->header;
		cur_pose_.pose.position = odom_msg->pose.pose.position;
	    inter_time_ = cur_pose_.header.stamp.nanosec - ex_time_;
        if(inter_time_ <= 0) inter_time_ += INTER_TIME_PLUS; 
        
	    ex_time_ = cur_pose_.header.stamp.nanosec;
	    ex_x_ = cur_pose_.pose.position.x;
        ex_y_ = cur_pose_.pose.position.y;
		is_pose_ = true;
		cout<<"is_pose_"<<is_pose_<<endl;
		tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
			odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll,pitch,yaw);
		filtered_yaw = yaw*(180.0 / M_PI);
		// cout<<" filtered_yaw "<< filtered_yaw <<endl;
		// cout<<"ex_x_" << ex_x_<<endl;
		// cout<<"ex_y_" << ex_y_<<endl;
		// cout<<"cur_pose_.pose.position" << cur_pose_.pose.position<<endl;
		//cout << "Current Pose is : " << cur_pose_.pose.position << endl;
		//cout << "----------------" << endl;

	}
	// void OdomCallback(const nav_msgs::Odometry::ConstSharedPtr &odom_msg) 
	// {
	// 	double xpos_ = odom_msg->pose.pose.position.x;
	// 	double ypos_ = odom_msg->pose.pose.position.y;
				
	// 	cur_pose_.x = xpos_;
	// 	cur_pose_.y = ypos_;
	// 	// speed^2  = liner x^2 + liner y^2
	// }

	void AutoCallback(const std_msgs::Int8::ConstSharedPtr &auto_msg)
	{
		auto_flag = auto_msg->data;
		if (auto_flag ==1){
			is_auto_=true;

		}
	}

	void LaneCallback(const waypoint_maker::msg::Lane::ConstSharedPtr &lane_msg) 
	{
		// waypoints_.clear();
		// vector<waypoint_maker::msg::Waypoint>().swap(waypoints_);
		// waypoints_ = lane_msg->waypoints;
		// waypoints_size_ = waypoints_.size();
		
		// cout<<waypoints_<<endl;
		// if (waypoints_size_ != 0) is_lane_ = true;
		waypoints_.clear();
		vector<waypoint_maker::msg::Waypoint>().swap(waypoints_);
		waypoints_ = lane_msg->waypoints;

		waypoints_size_ = waypoints_.size();
		if (waypoints_size_ != 0) is_lane_ = true;
		for (int i = 0; i <= waypoints_size_; i++) {
		}
		// cout<<"is_lane_"<<is_lane_<<endl;
		// // cout<< waypoints_.x<< ", y: " << waypoints_.y<<endl;
		// cout<<"waypoints_size_"<<waypoints_size_<<endl;

    }
		

	void TrafficSignCallback(const std_msgs::Bool::ConstSharedPtr& vision_msg)
	{
		vision_check_ = vision_msg->data;
	}
	
	void StateCallback(const coss_msgs::msg::Coss::ConstSharedPtr &state_msg)
	{
		state[0] = state_msg->state1;
		//state[1] = state_msg->state2;
		state[2] = state_msg->state3;
		state[3] = state_msg->state4;
		state[4] = state_msg->state5;
		state[5] = state_msg->state6;

		lidar_steer_ = state_msg->lidar_steer;
		is_right_ = state_msg->is_right;
		lidar_stop_flag_ = state_msg->lidar_stop_flag;
	}
	


	
	void ImuCallback(const sensor_msgs::Imu::ConstSharedPtr &imu_msg)
	{
		tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
			imu_msg->orientation.z, imu_msg->orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll,pitch,yaw);
		cur_course_ = yaw * (180.0 / M_PI);
		
		cout<<"yaw"<<cur_course_<<endl;
	}

	///camera_steer_2044_coss 빗자루
	void LineCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr &camera_msg)
	{
		camera_angle_ = camera_msg->drive.steering_angle;
		cout<<"camera steer - "<<camera_angle_<<endl;
	}
	//camera_steer_2044_coss 한라인
	void LeftLineCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr &camera_msg)
	{
		left_camera_angle_ = camera_msg->drive.steering_angle;
		// cout<<"one_camera_angle_ - "<<one_camera_angle_<<endl;
	}

	void RightLineCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr &camera_msg)
	{
		right_camera_angle_ = camera_msg->drive.steering_angle;
		if (coss_state == 1 && right_camera_angle_ > 0.19)
		{
			right_cam_count++;
			if(right_cam_count > 10){
				state[1] = true;
			}
		}
		else if(coss_state == 1 && right_camera_angle_ <= 0.3) right_cam_count = 0;
		// cout<<"one_camera_angle_ - "<<one_camera_angle_<<endl;
	}
	
	// void VerticalCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr &lidar_msg)
	// {
	// 	lidar_angle_ = lidar_msg->drive.steering_angle * -(M_PI / 180.0) * 2;
	// }


	void getClosestWaypoint(geometry_msgs::PoseStamped current_pose) 
	{
		if (!waypoints_.empty()) 
		{
			double dist_min = MAX_SEARCH_DIST;
			for (int i = 0; i < waypoints_.size(); i++)
			{
				double dist = calcPlaneDist(current_pose, waypoints_[i].pose);
				if (dist < dist_min) 
				{
					dist_min = dist;
					waypoint_min_ = i;
				}
			}
			closest_waypoint_ = waypoint_min_;
		}
		else cout << "------ NO CLOSEST WAYPOINT -------" << endl;
	}
	




	// double getSpeed(double& ex_x, double& ex_y, double cur_x, double cur_y)
	// {
    //     double distance = sqrt(pow((cur_x - ex_x ), 2) + pow((cur_y - ex_y), 2));
    //     double speed = distance / inter_time_ * INTER_SPEED_TIME_MAX;
 
    //     // cout << "distance   :: " << distance << endl;
    //     // cout << "time   "  << inter_time_ <<  "    ::    speed  " << speed << endl;
    //     return speed;
	// }
	
    double PID(double target, double cur_speed) 
	{
        if (target == 0) return 0;
        target = target - PID_FIT;
        double error = target - cur_speed;
        if (error < 0) sum_error_ = 0;
        
        sum_error_ += error;
        
        if (sum_error_ > MAX_SUM) sum_error_ = MAX_SUM;
        else if (sum_error_ < MIN_SUM) sum_error_ = MIN_SUM;
        
        double result = KI * sum_error_ + target;
        if (result >= 20.0) result = 20.0;
        
        return result;
    }

	void mission_state_control_(){
		if(state[0] == true && coss_state == 0)
		{
			coss_state = 1;
		}
		else if (state[1] == true && coss_state == 1)
		{
			coss_state = 2;
		}
		else if (state[2] == true && coss_state == 2)
		{
			coss_state = 3;
		}
		else if (stopline_ && coss_state == 3)
		{
			coss_state = 4;
		}
		else if (state[4] == true && coss_state == 4)	
		{
			coss_state = 5;
		}
		else if (state[5] == true && coss_state == 5)
		{
			coss_state = 6;
		}
		
		coss_msg_.mission_state = coss_state;
		state_pub_.publish(coss_msg_);
	}
    
	double PurePursuit();
	double Stanley();
	void process();
	int find_cloestindex(geometry_msgs::PoseStamped pose);
	double cross_error(geometry_msgs::PoseStamped pose);
	double calcAngularVelocity();
};

#endif
