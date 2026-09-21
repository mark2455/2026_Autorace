#include <ros/ros.h>
#include <waypointFollower.h>

double WaypointFollower::PurePursuit()
{
    
	getClosestWaypoint(cur_pose_);
    
    double t_dist = .0; 
    
	//cout<<"waypoints_size_"<<waypoints_size_<<endl;
    
    for(int i = 0; i < waypoints_size_; i++) 
    {
        
        double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
        
        if(dist > lookahead_dist_)
        {
            target_index_ = i;
            t_dist = dist;
            break;
        }
    }
    
    
	double steering_angle;
	double target_x = waypoints_[target_index_].pose.pose.position.x;
	double target_y = waypoints_[target_index_].pose.pose.position.y;
    double cur_x = cur_pose_.pose.position.x;
    double cur_y = cur_pose_.pose.position.y;
    double t_x = waypoints_[target_index_].pose.pose.position.x;
    double t_y = waypoints_[target_index_].pose.pose.position.y;
    double dx = t_x - cur_x;
    double dy = t_y - cur_y;
    
    double theta = atan2(dy, dx);
    double temp_theta = atan2(dy,dx) * 180.0/M_PI;
    
    double deg_alpha = (temp_theta - cur_course_);
    
    double alpha = deg_alpha * M_PI/180.0;

    double filtered_alpha = (temp_theta - filtered_yaw);
    double filtered_alpha_deg = filtered_alpha * M_PI/180.0;
    is_alpha = filtered_alpha_deg;
    cout<<"alpha=="<<is_alpha<<endl;
    
    return atan2((2.0* sin(alpha) / lookahead_dist_), 1.0);
}

double WaypointFollower::calcAngularVelocity()
{
    getClosestWaypoint(cur_pose_);

    double t_dist = .0;    // 현재 위치와 목표 웨이포인트 사이의 거리
    for (int i = 0; i < waypoints_size_; i++) 
    {
        double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
        if (dist > lookahead_dist_) 
        {
            target_index_ = i;
            t_dist = dist;
            break;
        }
    }
    
    double target_x = waypoints_[target_index_].pose.pose.position.x;
    double target_y = waypoints_[target_index_].pose.pose.position.y;
    double cur_x = cur_pose_.pose.position.x;
    double cur_y = cur_pose_.pose.position.y;
    double dx = target_x - cur_x;
    double dy = target_y - cur_y;

    // 로봇의 현재 방향과 목표 지점의 각도 차이
    double theta = atan2(dy, dx);
    double alpha = theta - cur_course_;
    double filtered_alpha = theta - filtered_yaw;
    // 각속도 계산
    double angular_velocity = -(2.0 * 0.2 * sin(alpha)) / lookahead_dist_;
    cout<<angular_velocity<<endl;
    return angular_velocity;
}

double WaypointFollower::Stanley()
{
    getClosestWaypoint(cur_pose_);

    double t_dist = .0;    // 현재 위치와 목표 웨이포인트 사이의 거리
    for (int i = 0; i < waypoints_size_; i++) 
    {
        double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
        if (dist > lookahead_dist_) 
        {
            target_index_ = i;
            t_dist = dist;
            break;
        }
    }
    
    double target_x = waypoints_[target_index_].pose.pose.position.x;
    double target_y = waypoints_[target_index_].pose.pose.position.y;
    double cur_x = cur_pose_.pose.position.x;
    double cur_y = cur_pose_.pose.position.y;

    // 로봇의 현재 위치에서 목표 웨이포인트까지의 거리
    double dx = target_x - cur_x;
    double dy = target_y - cur_y;

    // 로봇의 현재 방향과 목표 지점의 각도 차이
    double path_theta = atan2(dy, dx);
    double heading_error = path_theta - cur_course_;

    // 각도 차이를 -pi에서 pi 사이로 조정
    if (heading_error > M_PI)
        heading_error -= 2 * M_PI;
    else if (heading_error < -M_PI)
        heading_error += 2 * M_PI;

    // Cross-track error 계산
    double cross_track_error = dy * cos(cur_course_) - dx * sin(cur_course_);

    // Stanley Controller
    double k = 1.0; // Control gain
    double angular_velocity = heading_error + atan2(k * cross_track_error, speed_);
    return angular_velocity;
}







