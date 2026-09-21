#include <ros/ros.h>
#include <waypointFollower.h>

void WaypointFollower::process() 
{
	//cout<<"is_auto_ : "<<is_auto_<<endl;
	is_auto_msg_.data = is_auto_;
	auto_pub_.publish(is_auto_msg_);
	if(is_auto_ == true)
	{ 
		mission_state_control_();

		double cam_ang = left_camera_angle_;
		if(fabs(cam_ang) >= 0.34) cam_ang = 0.34;

		is_control_ = true;
		switch(coss_state)
		{
			case 0: //라이다------------------------
				//speed_ = 4.0;
				speed_ = 2.5;
				break;
			case 1: //차선+감속
				if (redzone_)
				{
					speed_ = 2.0;
				}
				else
				{
					speed_ = 2.0;
					//speed_ = 30/(1+exp(2.5*(fabs(cam_ang)-0.9)))-21.5;
				}
				if (!stoponce_flag)
				{
					if (grayzone_)
					{
						speed_ = 0.0;
						cur_steer = 0.0;
						control_msg_.drive.speed =speed_;
						control_msg_.drive.steering_angle = cur_steer;
						control_pub_.publish(control_msg_);
						stoponce_flag =true;
						ros::Duration(6.0).sleep();  //2초간 정지
						cout<<"---STOP FOR 3 SECONDS---"<<endl;
					}
				}
				break;
			// case 1:
			// 	if (redzone_)
			// 	{
			// 		speed_ = 2.0;
			// 	}
			// 	else
			// 	{
			// 		speed_ = 3.0;
			// 		//speed_ = 30/(1+exp(2.5*(fabs(cam_ang)-0.9)))-21.5;
			// 	}

			// case 2: //횡단보도에서 state -> 2  교차로+차단기
			// 	if (!stoponce_flag)
			// 	{
			// 		if (stopline_)
			// 		{
			// 			speed_ = 0.0;
			// 			cur_steer = 0.0;
			// 			control_msg_.drive.speed =speed_;
			// 			control_msg_.drive.steering_angle = cur_steer;
			// 			control_pub_.publish(control_msg_);
			// 			stoponce_flag =true;
			// 			ros::Duration(6.0).sleep();  //2초간 정지
			// 			cout<<"---STOP FOR 3 SECONDS---"<<endl;
			// 		}
			// 	}
			// 	else
			// 	{
			// 		if (lidar_stop_flag_)
			// 		{
			// 			speed_ = 0.0;
			// 		}
			// 		else
			// 		{
			// 			speed_ = 3.0;
			// 			//speed_ = 30/(1+exp(2.5*(fabs(cam_ang)-0.9)))-21.5;
			// 		}

			// 	}
				
			// 	break;
			case 2: //횡단보도에서 state -> 2  교차로+차단기
				if (lidar_stop_flag_)
				{
					speed_ = 0.0;
				}
				else
				{
					speed_ = 3.0;
					//speed_ = 30/(1+exp(2.5*(fabs(cam_ang)-0.9)))-21.5;
				}
				
				break;
			case 3: //터널
				// speed_ = 30/(1+exp(2.5*(fabs(cam_ang)-0.9)))-21.5;
				speed_ = 2.5;
				break;
			case 4: //Parking
				if (!stoponce_flag)
				{
					if (stopline_)
					{	
						stoponce_flag = true;
					}
					else
					{
						speed_ = 3.0;
						parking_steer_ = right_camera_angle_;
						//speed_ = 30/(1+exp(2.5*(fabs(cam_ang)-0.9)))-21.5;
					}
				}	
				else
				{
					if(go_count<= 10){
						speed_ = 3.0;
						parking_steer_ = -2.0;
						go_count++;
						if(go_count >= 9){
							is_stop = true;	
						}
					}
			
					if(is_stop == true)
					{
						speed_ = 0.0;
						parking_steer_ = 0.0;
						control_msg_.drive.speed =speed_;
						control_msg_.drive.steering_angle = cur_steer;
						control_pub_.publish(control_msg_);
						cout<<"---STOP FOR 2 SECONDS gear change ---"<<endl;
						ros::Duration(1.0).sleep();  //1초간 정지
						is_stop = false;
						is_rear = true;
					}
					
				}	
				
				if(is_rear == true)
				{

					if(rear_count<= 20)
					{	
						cout<<"rearearearearearae"<<endl;
						rear_count ++ ; 
						speed_ = -2.0;
						parking_steer_ = 0.4;
					}
						
					else if(rear_count<= 31 && rear_count>=21)
					{
						cout<<"rearearearearearae"<<endl;
						rear_count ++ ; 
						speed_ = -2.0;
						parking_steer_ = 0.0;

						
					}

					else if(rear_count<= 39 && rear_count>=32)
					{
						cout<<"rearearearearearae"<<endl;
						rear_count ++ ; 
						speed_ = -2.0;
						parking_steer_ = -1.3;

						
					}
					else
					{	
						if(is_rear_count<= 20)
						{
							parking_steer_ = 0.0;
							speed_ = 0.0;
							cout<<"-- gear change !!!!!!!!!!!!!!!!!!!!11---"<<endl;
							is_rear_count++;
						}
						else
						{
							is_front = true;
							is_rear = false; 
						}

					}

				}

				if(is_front == true) 
				{	
					if(front_count<= 11)
					{	
						speed_ = 2.0;	
						parking_steer_ = -1.3;
						front_count++;
						
					}

					else if(front_count<= 55 && front_count>=12)
					{
						speed_ = 2.0;	
						parking_steer_ = 1.3;
						front_count++;
						
					}
					else
					{
						is_front = false; 
					}
				}
				break;
			case 5: //미션 끝 정지
				speed_ = 0.0;
				break;
		
		}
		
	}

	if (is_control_) 
	{ 
		switch(coss_state)
		{
			case 0: // lidar_track
				// cur_steer = parking_steer_;
				cur_steer= lidar_steer_;
				path_tracking = "lidar";
				// cout<<"cur_steer--"<<cur_steer<<endl;
				break;
			case 1:
				if (redzone_)
				{
					cur_steer = 0.0;
					path_tracking = "offset";
				}
				cur_steer= right_camera_angle_;
				path_tracking = "camera";
				break;
			case 2:
				cur_steer= right_camera_angle_;
				path_tracking = "camera";
				// if (is_right_ == 0)
				// {
				// 	cur_steer= left_camera_angle_;
				// 	cout << "left" <<endl;
				// }
				// else if (is_right_ == 1)
				// {
				// 	cur_steer= right_camera_angle_;
				// 	cout << "right" <<endl;
				// }
				break;
			case 3:
				if(cam_count <= 10){
					cur_steer= right_camera_angle_;
					path_tracking = "camera";
					cam_count++;
				}
				else if(cam_count >= 11 && cam_count <= 105){
					cur_steer = lidar_steer_;
					path_tracking = "lidar";
					cam_count ++;
				}
				else{
					cur_steer = right_camera_angle_;
					path_tracking = "camera";
				}
				break;
			case 4:
				cur_steer = parking_steer_;
				path_tracking = "parking";
				break;
			case 5:
				cur_steer = 0.0;
				break;
			case 6:
				cur_steer = right_camera_angle_;
				break;
		}
		// if(speed_ > 4.0) speed_ = 4.0;
		// cout<<control_msg_.angular.z<<endl;
		control_msg_.drive.speed = speed_;
		control_msg_.drive.steering_angle = cur_steer;

		cout<<"-----Follower-----"<<endl;
		cout<<"is_auto_ : "<<is_auto_<<endl;
		cout<<"coss_state :"<<coss_state<<endl;
		cout<<"redzone_ : "<<redzone_<<endl;
		cout<<"speed_ : "<<speed_<<endl;
		cout<<"cur_steer : "<<cur_steer<<endl;
		std::cout << "path_tracking : "<<path_tracking<< std::endl;
		std::cout << "\n";
		
		control_pub_.publish(control_msg_);
		
	}
	
	is_auto_ = false; 
	// cout << "IS DYNAMIC FINISHED    ::    "   << is_dynamic_finished_ << endl;
	// cout << "CUR_SPEED  ::   "  << cur_speed_ << endl;
    //cout << "-------------------------------- " << endl;
	
}

