// //------------꼬깔 적은 버전------------------------|

// #include "lidar_lib/lidar_lib.h"
// #include "lidar_lib/cubic_spline_planner.h"
// #include <coss_msgs/msg/coss.hpp>

// bool track_cmp(const PointType &p1, const PointType &p2){
//   return p1.x < p2.x;
// }
// bool duplicates_track_cmp(int a, int b){
//   return a < b;
// }

// class Task1 : public Lidar_Func{
// private:
// //-------------param-------------------------|
//   float voxel_size;
//   float roi[4];
//   float param[3];
//   float speed_total_;
//   double first_l, first_r;
//   double cone_dist_;
//   double dist_offset_;
//   int before_angle;
//   int steer_total_;
//   int current_speed_;
//   int target_index_;

//   bool lidar_end;
//   int lidar_count;
//   float prev_steer;

// //--------------vector-----------------------|
//   vector<PointType> center;
//   vector<PointType> outline_cones;
//   vector<PointType> inline_cones;
//   vector<int> center_index;
//   vector<double> r_rx, r_ry, r_ryaw, r_rk, l_rx, l_ry, l_ryaw, l_rk;
//   vector<double> avg_vec_;

// //-----------visual waypoint-----------------|
//   geometry_msgs::Point wp;
//   coss_msgs::msg::Coss coss_msg;

// //-------------PCL---------------------------|
//   pcl::PointCloud<PointType>::Ptr msg_;
//   sensor_msgs::PointCloud2::SharedPtr msg;
// //-------------PUB---------------------------|

//   ros::Publisher pub_path_l;
//   ros::Publisher pub_path_r;
//   ros::Publisher cmd_vel;
//   ros::Publisher end_pub_;

// //-------------SUB---------------------------|
//   ros::Subscriber sub_points;

// public:
//     Task1(){
//       initsetup();
//       msg.reset(new sensor_msgs::PointCloud2());
//       msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
//     }
//     ~Task1(){
//       ROS_INFO("Task1 Done");
//     }

//     void initsetup();
//     void run();
//     void scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan);
//     void vector_clear();
//     void decision();
//     void make_line();
//     bool is_in_vector(vector<int> v, int element);
//     void make_waypoint();
//     int GetAverage(vector<double> const& vec);
//     double pure_pursuit(
//       vector<double>& rx,
//       vector<double>& ry,
//       geometry_msgs::PoseStamped& target_p,
//       float& speed,
//       vector<double>& rk
//     );
// };

//   void Task1::scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan){
//       laser_geometry::LaserProjection projector_;
//       projector_.projectLaser(*scan, *msg);
//       (*msg_).width = (*msg).points.size();
//       (*msg_).height = 1;
//       (*msg_).points.resize((*msg_).width * (*msg_).height);
//       // 앞 뒤 바뀌면 x, y에 - add
//     for (int i = 0; i < (*msg).points.size(); i++)
//     {
//       (*msg_).points[i].x = (*msg).points[i].x;
//       (*msg_).points[i].y = (*msg).points[i].y;
//       (*msg_).points[i].z = 0;
//     }
//   }
// //-----------------DATA 전처리-----------------------|
//   void Task1::initsetup(){
//     ros::param::get("/Task1/min_x_", roi[0]);
//     ros::param::get("/Task1/max_x_", roi[1]);
//     ros::param::get("/Task1/min_y_", roi[2]);
//     ros::param::get("/Task1/max_y_", roi[3]);
//     ros::param::get("/Task1/voxel_size", voxel_size);
//     ros::param::get("/Task1/tolerance", param[0]);
//     ros::param::get("/Task1/cluster_size_min", param[1]);
//     ros::param::get("/Task1/cluster_size_max", param[2]);
//     ros::param::get("/Task1/cone_dist", cone_dist_);
//     ros::param::get("/Task1/dist_offset", dist_offset_);

//   //--------------PUB--------------------------------------|
//     cmd_vel = nh_.advertise<geometry_msgs::msg::Twist>("ackermann_steering_controller/cmd_vel", 10);
//     end_pub_ = nh_.advertise<coss_msgs::msg::Coss>("coss_state", 1);

//     pub_path_l = nh_.advertise<nav_msgs::Path>("left_path", 1);
//     pub_path_r = nh_.advertise<nav_msgs::Path>("right_path", 1);

//   //--------------SUB---------------------------------------|
//     sub_points = nh_.subscribe("/scan", 1, &Task1::scanCallback, this);

//   //--------------Index 초기화-------------------------------|
//     first_l = 2.0;
//     first_r = -2.0;

//     before_angle = 0;

//     current_speed_ = 0;

//     target_index_ = 0;

//     lidar_end = false;

//     lidar_count = 0;

//     prev_steer = .0;

//     r_rx.resize(1);
//     r_ry.resize(1);
//     r_ryaw.resize(1);
//     r_rk.resize(1);

//     l_rx.resize(1);
//     l_ry.resize(1);
//     l_ryaw.resize(1);
//     l_rk.resize(1);
//   }

//   void Task1::vector_clear(){
//     center.clear();
//     outline_cones.clear();
//     inline_cones.clear();
//     center_index.clear();
    
//     r_rx.clear();
//     r_ry.clear();
//     r_ryaw.clear();
//     r_rk.clear();

//     l_rx.clear();
//     l_ry.clear();
//     l_ryaw.clear();
//     l_rk.clear();
//   }
  
//   //----------------CONE CONE CONE CONE CONE CONE CONE CONE CONE CONE------------------------|
//   void Task1::decision(){
//     vector<vector<double>> cloud_l, cloud_r;
//     vector<double> x, y, rx, ry, ryaw, rk; 

//     int first_left_index = 0, first_right_index = 0;
//     double yy;
//     bool left_flag = false, right_flag = false;

//     for(int i = 0; i < center.size();i++){  // index 넣어주기
//       center_index.push_back(i);
//     }

//     for (size_t i = 0; i < center_index.size(); i++)  // 좌,우 판단 후 index 넣어주기
//     {
//       PointType point;

//       yy = center[center_index[i]].y;

//       if(yy > 0 && !left_flag){           // y좌표로 좌우 판단  
//         left_flag = true;
//         first_left_index = i;             // index 넣어주기(최초 1번만 실행)
//       }
//       else if (yy < 0 && !right_flag){
//         right_flag = true;
//         first_right_index = i;
//       }
//       if(left_flag && right_flag){
//         break;
//       }
//     }

//     vector<int> left_index;
//     left_index.push_back(first_left_index);         // 처음 점 넣기
//     for(int i = 0; i < center_index.size(); i++){
//       if(is_in_vector(left_index,i)){               // left_index에 i 가 있으면 ture, 없으면 false
//         continue;
//       }
//       double temp_x = center[left_index.back()].x;
//       double temp_y = center[left_index.back()].y;
//       int compare = left_index.back();
//       int index = 0;
//       double min_dist = 9999;
//       for(int j = compare; j < center_index.size(); j++){
//         double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
//         if(min_dist > now_dist && !is_in_vector(left_index,j)){   // 가장 가까운 거리 index 추출
//           min_dist = now_dist;
//           index = j;
//         }
//       }
//       if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){ // 왼쪽 점과 가까운 점의 거리가 3.0 보다 크면(오른쪽 점)
//         break;
//       }
//       else{
//         left_index.push_back(index);
//       }
//     }

//     vector<int> right_index;
//     right_index.push_back(first_right_index);
//     for(int i = 0; i < center_index.size(); i++){
//       if(is_in_vector(right_index,i)){
//         continue;
//       }
//       double temp_x = center[right_index.back()].x;
//       double temp_y = center[right_index.back()].y;
//       int compare = right_index.back();
//       int index = 0;
//       double min_dist = 9999;
//       for(int j = compare; j < center_index.size(); j++){
//         double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
//         if(min_dist > now_dist && !is_in_vector(right_index,j)){
//           min_dist = now_dist;
//           index = j;
//         }
//       }
//       if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){
//         break;
//       }
//       else{
//         right_index.push_back(index);
//       }
//     }
//     vector<int> duplicates;
// 	  if(center.size()!=0){
// 	  	for (int value : left_index) { // left index의 요소를 value에 할당(left index의 요소 수 만큼 반복)
// 	  		if (std::find(right_index.begin(), right_index.end(), value) != right_index.end()) {  // 중복 index 확인
// 	  			duplicates.push_back(value);  // duplicates : 좌우 중복으로 분류된 index
// 	  		}
// 	  	}
// 	  	sort(duplicates.begin(), duplicates.end(),duplicates_track_cmp);  //  작은 숫자부터 정렬
// 	  	float min_dist = 9999;
// 	  	int min_index;
// 	  	for(int i = 0; i < duplicates.size();i++){
// 	  		vector<int> min_duplicates;
// 	  		int value = duplicates[i];  // value : 좌우 중복으로 분류된 index 값
// 	  		if(fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) > 1.2){ // 중복된 index의 좌표가 오른쪽 보다 왼쪽 첫 점과 거리 차이가 많이 나면서 1.2 초과일 때(오른쪽 점일 때)
// 	  			left_index.erase(std::remove(left_index.begin(), left_index.end(), value), left_index.end());                                     // 왼쪽에서 지워줌
// 	  			cout << fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) << endl;
// 	  			cout << "index " << duplicates[i] << "right" << endl;
// 	  		}
// 	  		else if (fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) < -1.2){  // 중복된 index의 좌표가 왼쪽 보다 오른쪽 첫 점과 거리 차이가 많이 나면서 -1.2 미만일 때(왼쪽 점일 때)
// 	  			right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end());                                         // 오른쪽에서 제거
// 	  			cout << fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) << endl;
// 	  			cout << "index " << duplicates[i] << "left" << endl;
// 	  		} 
// 	  		else if(fabs(fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y)) <= 1.2){ // 중복인데 거리 차이가 1.2 이하일 때
// 	  			cout << "what!!!!!?????" << endl;
// 	  			cout << "index " << duplicates[i] << endl;
// 	  			for(int j=0; j < value; j++){
// 	  				if(min_dist > sqrt(pow(center[duplicates[i]].x - center[center_index[j]].x,2) + pow(center[duplicates[i]].y - center[center_index[j]].y,2))){ // 중복된 점 중 거리가 1.2 이하인 가장 가까운 점 찾기
// 	  					min_dist = sqrt(pow(center[duplicates[i]].x - center[center_index[j]].x,2) + pow(center[duplicates[i]].y - center[center_index[j]].y,2));
// 	  					min_index = j;    // 중복된 것 중에서 1.2 이하인 인덱스 넣기
// 	  				}
//  	  			}
// 	  			cout << "min dist: " << min_dist << endl; 
// 	  			cout << "min index: " << min_index << endl;
// 	  			if (std::find(right_index.begin(), right_index.end(), min_index) != right_index.end()) {  // min_index가 right_index에 있을 때
// 	  			    min_duplicates.emplace_back(min_index);
// 	  			}

// 	  			if(min_duplicates.size() == 0) {
// 	  				right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end()); // value값(전체 index 중 중복된 index) 찾아서 제거
// 	  				cout << "hahah this is left"<< endl;
// 	  			}
// 	  			else if(min_duplicates.size() != 0){
// 	  				left_index.erase(std::remove(left_index.begin(), left_index.end(), value), left_index.end()); // value값(전체 index 중 중복된 index) 찾아서 제거
// 	  				cout << "hahah this is right" << endl;
// 	  			}
// 	  			min_dist = 9999;
// 	  		}
// 	  	}
// 	  }
  


  
//     if(center.size() > 0){
//       for(int i = 0; i < left_index.size();i++){          // 왼쪽 cone 좌표 넣기
//         outline_cones.push_back(center[left_index[i]]);
//       }
//       for(int i = 0; i < right_index.size();i++){         // 오른쪽 cone 좌표 넣기
//         inline_cones.push_back(center[right_index[i]]);
//       }
//     }
//     sort(outline_cones.begin(),outline_cones.end(),track_cmp);  // x좌표 기준 정렬
//     sort(inline_cones.begin(),inline_cones.end(),track_cmp);    // x좌표 기준 정렬
//     cout << "center size       : " << center.size() << endl;
//     cout << "inline cone size  : " << inline_cones.size() << endl;
//     cout << "outline cone size : " << outline_cones.size() << endl;
//   }
  

//   void Task1::make_line() {
//     vector<double> input_x_l, input_y_l,input_x_r,input_y_r;
//     input_x_l.clear();
//     input_y_l.clear();
//     input_x_r.clear();
//     input_y_r.clear();
//     double ds = 0.1;

//     if(outline_cones.size() > 0){ // 처음 점 임의로 생성
//         input_x_l = {-0.5};
//         input_y_l = {first_l};  // first_l = 2.0
//     }
//     if(inline_cones.size() > 0){
//         input_x_r = {-0.5};
//         input_y_r = {first_r};  // first_r = -2.0
//     }


//     for (size_t i = 0; i < outline_cones.size(); i++) // 최종 왼쪽 좌표 넣어주기
//     {
//       double x,y;

//       x = outline_cones[i].x;
//       y = outline_cones[i].y;
//       input_x_l.push_back(x);
//       input_y_l.push_back(y);

//     }

//     for (size_t i = 0; i < inline_cones.size(); i++) // 최종 오른쪽 좌표 넣어주기
//     {
//       double x,y;
//       x = inline_cones[i].x;
//       y = inline_cones[i].y;
      
//       input_x_r.push_back(x);
//       input_y_r.push_back(y);
//     }


//     if (outline_cones.size() <= 1){//예외처리 코드
//       cout << "왼쪽 방어코드 실행" << endl;
//       input_x_l.clear();
//       input_y_l.clear();

//       if(inline_cones.size() >=3){
//         for(int i = 0; i < input_x_r.size(); i++){  // 오른쪽 점 밀어서 왼쪽에 사용
//             input_x_l.push_back(input_x_r[i]);
//             input_y_l.push_back(input_y_r[i] + dist_offset_);
//         }  
        
//       }
//       else{ // 임의로 넣어줌
//         input_x_l.resize(2);
//         input_y_l.resize(2);
//         input_x_l = {0.0,4.0};
//         input_y_l = {2.0,2.0};
//       }
//       calc_spline_course(input_x_l,input_y_l,l_rx,l_ry,l_ryaw,l_rk,ds); // 출력(좌표) : l_rx,l_ry
//     }
//     else{
//       calc_spline_course(input_x_l,input_y_l,l_rx,l_ry,l_ryaw,l_rk,ds);
//     }  

//     if (inline_cones.size() <= 1){//예외처리 코드
//       cout << "오른쪽 방어코드 실행" << endl;
//       input_x_r.clear();
//       input_y_r.clear();

//       if(outline_cones.size() >=3){
//         for(int i = 0; i < input_x_l.size(); i++){  // 왼쪽 점 밀어서 오른쪽에 사용
            
//             input_x_r.push_back(input_x_l[i]);
//             input_y_r.push_back(input_y_l[i] - dist_offset_);
//         }
        
//       }
//       else{ // 임의로 넣어줌
//         input_x_r.resize(2);
//         input_y_r.resize(2);
//         input_x_r = {0.0,4.0};
//         input_y_r = {-2.0,-2.0};
//       }
//       calc_spline_course(input_x_r,input_y_r,r_rx,r_ry,r_ryaw,r_rk,ds);
//     }
//     else{
//       calc_spline_course(input_x_r,input_y_r,r_rx,r_ry,r_ryaw,r_rk,ds);
//     }

//     nav_msgs::Path path_msg_l;
//     nav_msgs::Path path_msg_r;

//     path_msg_l.header.stamp = ros::Time::now();
//     path_msg_l.header.frame_id = frame_id;
//     for (size_t i = 0; i < l_rx.size(); i++)
//     {
//         geometry_msgs::PoseStamped pose;
//         pose.pose.position.x = l_rx[i]; // spline 돌린 값 넣어줌
//         pose.pose.position.y = l_ry[i];
//         pose.pose.orientation.w = 1.0;
//         path_msg_l.poses.push_back(pose);
//     }
//     pub_path_l.publish(path_msg_l);

//     path_msg_r.header.stamp = ros::Time::now();
//     path_msg_r.header.frame_id = frame_id;
//     for (size_t i = 0; i < r_rx.size(); i++)
//     {
//         geometry_msgs::PoseStamped pose;
//         pose.pose.position.x = r_rx[i];
//         pose.pose.position.y = r_ry[i];
//         pose.pose.orientation.w = 1.0;
//         path_msg_r.poses.push_back(pose);
//     }
//     pub_path_r.publish(path_msg_r);


//     if (r_ry.size() >= 3){  // 첫 점 3개 묶어서 사용(평균값), 2번째 루프부터
//       first_r = 0;
//       first_r += (r_ry[0] + r_ry[1] + r_ry[2]);
//       first_r /= 3;
//     }
//     if (l_rx.size() >= 3){
//       first_l = 0;
//       first_l += (l_ry[0] + l_ry[1] + l_ry[2]);
//       first_l /= 3;
//     }
//   }

//   bool Task1::is_in_vector(vector<int> v, int element){
//       vector<int>::iterator it;                 // vector 반복자 it 선언
//       it = find(v.begin(), v.end(), element);   // element를 찾을 때 까지 반복
//       if (it != v.end()) {                      // 끝까지 찾았을 때 찾은 위치가 end가 아니면(element가 있으면)
//           return true;
//       } else {
//           return false;
//       }
//   }

//   void Task1::make_waypoint(){    //size = (int)(rx.size()*0.5);
//     vector<double> input_x, input_y,rx,ry,ryaw,rk;
//     double ds = 0.1;
//     double angle = .0;
//     float speed = 0.0f;
//     int size_ = (l_rx.size() >= r_rx.size()) ? r_rx.size() : l_rx.size(); //왼쪽, 오른쪽 중 작은 사이즈를 선택
//     int count = 0;
//     double sum_x = 0, sum_y = 0;
//     int size;

//     // state 넘기
//     if(inline_cones.size() <= 1 && outline_cones.size() <= 1) {
//       lidar_count++;
      
//       // cout << "prev_steer : " << prev_steer << endl;

//       if(lidar_count >= 5) {
//         coss_msg.state1 = true;
//         lidar_count = 0;
//       }
//       end_pub_.publish(coss_msg);
//     }
//     else if(inline_cones.size() > 1 || outline_cones.size() > 1) lidar_count = 0;
//     //중앙경로 생성
//     for (int i = 0; i < size_; i++){
//       if(count%3 == 0){
//         sum_x += (l_rx[i] + r_rx[i])/2; // 좌우 평균값
//         sum_y += (l_ry[i] + r_ry[i])/2;
//         // 3 << "sum_x : : " << sum_x << endl;
//         input_x.push_back(sum_x/3); // 3개씩 묶어서 평균값
//         input_y.push_back(sum_y/3);
//         sum_x = 0;
//         sum_y = 0;
//       }
//       else{
//         sum_x += (l_rx[i] + r_rx[i])/2;
//         sum_y += (l_ry[i] + r_ry[i])/2;
//       }
//       count++;
//     }

//     if(inline_cones.size() > 0 && outline_cones.size() >0){
//       input_x.push_back((l_rx[l_rx.size()-1] + r_rx[r_rx.size()-1])/2); // 마지막 인덱스 값 평균
//       input_y.push_back((l_ry[l_ry.size()-1] + r_ry[r_ry.size()-1])/2);
//     }

//     input_x.erase(input_x.begin());
//     input_y.erase(input_y.begin());

//     if (input_x.size() < 2){//예외처리 코드
//       input_x = {0.0,0.6};  // 임의로 넣어줌
//       input_y = {0.0,0.0};
//       calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
//     }
//     else{
//       calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
//     }

//     visualize_path(rx, ry, frame_id);

//     size = (int)(rx.size()*0.5);

//   // --------------------------주행-------------------------------------------------------------------------//
//     //cout << "waypoint size" << rx.size() << endl
//   // -----------------------------------

//     geometry_msgs::PoseStamped target_p;

//     if (rx.size() > 10) // 속력 설정
//     {
//     //------------define angle---------
//       //angle = pure_pursuit(rx, ry, target_p,speed);
//       // angle = stanley(rx,ry,ryaw,rk);
//       angle = pure_pursuit(rx, ry, target_p,speed,rk);
//       cout << "!!!!!!!!" << angle << endl;

//       before_angle = GetAverage(avg_vec_);

//       speed_total_ = 0.2;
//     }
//     else{
//       cout << "방어 steer" << endl;
//       angle = before_angle;

//       speed_total_ = 0.2;
//     }

//     if(sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2))< 0.1){
//       angle = before_angle;
//     }

//     //4.범위 이외시 최대조향각
//     if (angle >= 0.9)
//       angle = 0.9;
//     if (angle <= -0.9)
//       angle = -0.9;

//     //speed = return_speed(angle,rx,ry,rk); 
//     // speed_avg_vec_.push_back(speed);
//     // if (speed_avg_vec_.size() > 5){speed_avg_vec_.erase(speed_avg_vec_.begin());}
//     // speed_total_ = GetAverage(speed_avg_vec_);

//     // double task1_steer = angle / 85.3f;
//     double task1_steer = angle;
//     cout << "ANGLEEEEEEEEEEEEEEEEEE: " << task1_steer << endl;
//     // steer_total_ = task1_steer;

//     coss_msg.lidar_steer = task1_steer;
//     end_pub_.publish(coss_msg);


//     cout << "speed_total_  = " << speed_total_ << endl; 
//     cout << "steer_total_  = " << task1_steer << endl;
//     cout << "rx.size= " << rx.size() <<endl;
//     cout << "\n" <<endl;

//     avg_vec_.push_back(angle);
    
//     if(avg_vec_.size() > 5){ avg_vec_.erase(avg_vec_.begin());}

//   }

//   int Task1::GetAverage(vector<double> const& vec){
//       if (vec.empty()){
//           return 0;
//       }
//       return accumulate(vec.begin(),vec.end(),0.0) / vec.size();
//   }
//   double Task1::pure_pursuit(vector<double>& rx, vector<double>& ry, geometry_msgs::PoseStamped& target_p,float& speed,vector<double>& rk)
//   {
//     // 1. 목표 추종점 찾기
//     //float lookahead_dist = speed* 0.2 + 0.5; // temp_speed = m/s
//     // float lookahead_dist = 5.5;

//     // #### LD 피팅 ####
//   // 1안
//       // float lookahead_dist = 0.03*(0.03*current_speed_*current_speed_*current_speed_+7.5*current_speed_)+2;
//   // 2안
//       // lookahead_dist_ = tanf(0.08*cur_speed_ - 0.09)+4.0;
//       float lookahead_dist = 0.9f;

//       geometry_msgs::PoseStamped local_pose;
//       local_pose.pose.position.x = 0.0;
//       local_pose.pose.position.y = 0.0;    
      
//       //int min_idx = find_cloestindex(local_pose,rx,ry);
//       // double ratio_gain = 0;
//       // if (rk.size() > 11) ratio_gain =abs(rk[10]);
//       // else ratio_gain = abs(rk.back());
//       // if(ratio_gain > 0.2) lookahead_dist = 4.5;

//     float final_ld, alpha;
//     float cur_x = -0.25f; // 휠베이스
//     float cur_y = 0.0f;
//     float dist = 0.0f;
    
//     for (int i = 0; i < rx.size(); i++){
//       {
//         dist = sqrt(pow(rx[i]-cur_x,2) + pow(ry[i]-cur_y,2));
//         if (dist > lookahead_dist)
//         {
//           target_index_ = i;
//           final_ld = dist; // 최종으로 사용할 ld
//           break;
//         }
//         if (i == rx.size())
//         {
//           // cout << "30개 이상일 때 방어 " << endl;
//           target_index_ = i;
//           final_ld = dist;
//           break;
//         }
//       }
//       target_p.pose.position.x = rx[target_index_];
//       target_p.pose.position.y = ry[target_index_];
//     }
//     visualize_marker_geo(&target_p, frame_id, 1);
//     double x = target_p.pose.position.x + abs(cur_x);
//     double y = target_p.pose.position.y;

//     // 2. alpha 구하기 
//     alpha = -1*atan2f(y,x); // 일반적으로 아는 축으로 맞춤
//     // if(alpha < -90.0 && alpha > -180.0f) alpha += 270.0f;   //지우는게 맞는듯...?
//     // else alpha -= 90.0f;
//     // alpha = -alpha*M_PI/180.0f;
    
//     //3. 최종 조향각
//     if (final_ld > 0.9f)
//       final_ld = 0.9f;
//     if (final_ld < 0.2f)
//       final_ld = 0.2f;

//     float cur_steer = atan2f(2.0f * 0.25f * sinf(alpha) / (final_ld), 1.0f) * 180.0f / M_PI ; 
//     if(cur_steer>0) cur_steer *= -0.05;
//     else if(cur_steer<0) cur_steer *= -0.05;
//     cout << "===== Pure-pursuit ====="<< endl;
//     cout << "target_index_" << target_index_<<endl;
//     cout << "lookahead distance   : " << lookahead_dist <<endl;
//     cout << "final ld             : " << final_ld <<endl;
//     cout << "pure pursuit steer   : " << cur_steer << endl;
//     cout << "steer: "<< coss_msg.lidar_steer<<endl;
//     cout << "dist: " << sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2)) <<endl;

//     return cur_steer;
//   }

//   void Task1::run(){
//       vector_clear();
//       set_roi(msg_, roi);
//       voxel(msg_, voxel_size);
//       clustering(msg_, center, param);
//       cout << "center_size : " << center.size() << endl;
//       decision();
//       // visualize_rviz(center, frame_id , 0);
//       rviz_clear(frame_id);
//       visualize_rviz(outline_cones, frame_id , 1);
//       visualize_rviz(inline_cones, frame_id , 2);
//       make_line();
//       make_waypoint(); 
    
//   }

#include "lidar_lib/lidar_lib.h"
#include "lidar_lib/cubic_spline_planner.h"

bool track_cmp(const PointType &p1, const PointType &p2){
    if(p1.x < 1 && p2.x < 1){
      return p1.x < p2.x;
    }
    else{
      return p1.z < p2.z;
    }
}
bool duplicates_track_cmp(int a, int b) {
    return a < b; // 오름차순 정렬
}
class Task1 : public Lidar_Func{
private:                                                                                                                                                                                                                                                            
// param
    float voxel_size;
  	float roi[4];
  	float param[3];
    double first_l, first_r;
    double dist_offset;
    double cone_dist_;
    float before_angle;
    float steer_total_;

    int target_index_;
    int done_count;
    int done_count_param;
    int done_size;
    std_msgs::Int32 track_done_flag;
    std_msgs::Float64 SteerAngle;

// vector
    vector<PointType> center;
    vector<PointType> min_max_y;
    vector<PointType> outline_cones;
    vector<PointType> inline_cones;
    vector<int> center_index;
    vector<double> r_rx,r_ry,r_ryaw,r_rk,l_rx,l_ry,l_ryaw,l_rk;
    vector<double> avg_vec_;

// visual waypoint
    geometry_msgs::Point wp;

// pcl
    sensor_msgs::PointCloud2::SharedPtr msg;
    pcl::PointCloud<PointType>::Ptr msg_;

// pub, sub
    ros::Subscriber sub_points;
    ros::Publisher pub_steering_angle;
    ros::Publisher pub_path_l;
    ros::Publisher pub_path_r;
    ros::Publisher pub_flag;
    
public:
    Task1(){
        initsetup();
  	  	msg.reset(new sensor_msgs::PointCloud2());
        msg_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    ~Task1()
  	{
		ROS_INFO("Task1 Done");
  	}

    void initsetup();
    void run();
    void scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan);
    void vector_clear();
    void decision();
    void make_line();
    bool is_in_vector(vector<int> v, int element);
    void make_waypoint();
    int GetAverage(vector<double> const& vec);
    float pure_pursuit(vector<double>& rx, vector<double>& ry, geometry_msgs::PoseStamped& target_p,float& speed,vector<double>& rk);
};

void Task1::scanCallback(const sensor_msgs::LaserScan::ConstSharedPtr &scan) {
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, *msg);
    pcl::fromROSMsg(*msg, *msg_);
}

void Task1::initsetup(){
    ros::param::get("/track/min_x_", roi[0]);
	  ros::param::get("/track/max_x_", roi[1]);
	  ros::param::get("/track/min_y_", roi[2]);
	  ros::param::get("/track/max_y_", roi[3]);
    ros::param::get("/track/voxel_size", voxel_size);
	  ros::param::get("/track/tolerance", param[0]);
	  ros::param::get("/track/cluster_size_min", param[1]);
	  ros::param::get("/track/cluster_size_max", param[2]);
    ros::param::get("/track/cone_dist", cone_dist_);
    ros::param::get("/track/dist_offset", dist_offset);
    ros::param::get("/track/done_count_param", done_count_param);
    ros::param::get("/track/done_size", done_size);

    pub_steering_angle = nh_.advertise<std_msgs::Float64>("lidar_", 10);
    
    sub_points = nh_.subscribe("/scan", 1, &Task1::scanCallback, this);
    
    pub_path_l = nh_.advertise<nav_msgs::Path>("left_path",1);
    pub_path_r = nh_.advertise<nav_msgs::Path>("right_path",1);

    pub_flag = nh_.advertise<std_msgs::Int32>("mission_lidar",1);

    first_l = 0.25, first_r = -0.25, before_angle = 0, target_index_ = 0;
    r_rx.resize(1);r_ry.resize(1);r_ryaw.resize(1);r_rk.resize(1);
    l_rx.resize(1);l_ry.resize(1);l_ryaw.resize(1);l_rk.resize(1);
    
    done_count = 0, track_done_flag.data = -1;
}

void Task1::vector_clear(){
    center.clear();
    min_max_y.clear();
    outline_cones.clear();
    inline_cones.clear();
    center_index.clear();
    r_rx.clear();
    r_ry.clear();
    r_ryaw.clear();
    r_rk.clear();
    l_rx.clear();
    l_ry.clear();
    l_ryaw.clear();
    l_rk.clear();
}

void Task1::decision(){

    vector<vector<double>> cloud_l, cloud_r;
    vector<double> x, y, rx, ry, ryaw, rk; 

    int first_left_index = 0, first_right_index = 0;
    double yy;
    bool left_flag = false, right_flag = false;

    for(int i = 0; i < center.size();i++){
      center_index.push_back(i);
    }

    for (size_t i = 0; i < center_index.size(); i++)
    {
      PointType point;

      yy = center[center_index[i]].y;

      if(yy > 0 && !left_flag){
        left_flag = true;
        first_left_index = i;
      }
      else if (yy < 0 && !right_flag){
        right_flag = true;
        first_right_index = i;
      }
      if(left_flag && right_flag){
        break;
      }
    }

    vector<int> left_index;
    left_index.push_back(first_left_index);
    for(int i = 0; i < center_index.size(); i++){
      if(is_in_vector(left_index,i)){
        continue;
      }
      double temp_x = center[left_index.back()].x;
      double temp_y = center[left_index.back()].y;
      int index = 0;
      double min_dist = 9999;
      for(int j = 0; j < center_index.size(); j++){
        double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
        if(min_dist > now_dist && !is_in_vector(left_index,j)){
          min_dist = now_dist;
          index = j;
        }
      }
      if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){
        break;
      }
      else{
        left_index.push_back(index);
      }
    }

    vector<int> right_index;
    right_index.push_back(first_right_index);
    for(int i = 0; i < center_index.size(); i++){
      if(is_in_vector(right_index,i)){
        continue;
      }
      double temp_x = center[right_index.back()].x;
      double temp_y = center[right_index.back()].y;
      int index = 0;
      double min_dist = 9999;
      for(int j = 0; j < center_index.size(); j++){
        double now_dist = sqrt(pow(temp_x - center[center_index[j]].x,2) + pow(temp_y - center[center_index[j]].y,2));
        if(min_dist > now_dist && !is_in_vector(right_index,j)){
          min_dist = now_dist;
          index = j;
        }
      }
      if(sqrt(pow(temp_x - center[center_index[index]].x,2) + pow(temp_y - center[center_index[index]].y,2)) > cone_dist_){
        break;
      }
      else{
        right_index.push_back(index);
      }
    }
    vector<int> duplicates;
	  if(center.size()!=0){
	  	for (int value : left_index) {
	  		if (std::find(right_index.begin(), right_index.end(), value) != right_index.end()) {
	  			duplicates.push_back(value);
	  		}
	  	}
	  	sort(duplicates.begin(), duplicates.end(),duplicates_track_cmp);
	  	float min_dist = 9999;
	  	int min_index;
	  	for(int i = 0; i < duplicates.size();i++){
	  		vector<int> min_duplicates;
	  		int value = duplicates[i];
	  		if(fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) > 1.2){
	  			left_index.erase(std::remove(left_index.begin(), left_index.end(), value), left_index.end());
	  			cout << fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) << endl;
	  			cout << "index " << duplicates[i] << "right" << endl;
	  		}
	  		else if (fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) < -1.2){
	  			right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end());
	  			cout << fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y) << endl;
	  			cout << "index " << duplicates[i] << "left" << endl;
	  		} 
	  		else if(fabs(fabs(center[first_left_index].y - center[duplicates[i]].y) - fabs(center[first_right_index].y - center[duplicates[i]].y)) <= 1.2){
	  			cout << "what!!!!!?????" << endl;
	  			cout << "index " << duplicates[i] << endl;
	  			for(int j=0; j < value; j++){
	  				if(min_dist > sqrt(pow(center[duplicates[i]].x - center[center_index[j]].x,2) + pow(center[duplicates[i]].y - center[center_index[j]].y,2))){
	  					min_dist = sqrt(pow(center[duplicates[i]].x - center[center_index[j]].x,2) + pow(center[duplicates[i]].y - center[center_index[j]].y,2));
	  					min_index = j;
	  				}
 	  			}
	  			cout << "min dist: " << min_dist << endl; 
	  			cout << "min index: " << min_index << endl;
	  			if (std::find(right_index.begin(), right_index.end(), min_index) != right_index.end()) {
	  			    min_duplicates.emplace_back(min_index);
	  			}

	  			if(min_duplicates.size() == 0) {
	  				right_index.erase(std::remove(right_index.begin(), right_index.end(), value), right_index.end());
	  				cout << "hahah this is left"<< endl;
	  			}
	  			else if(min_duplicates.size() != 0){
	  				left_index.erase(std::remove(left_index.begin(), left_index.end(), value), left_index.end());
	  				cout << "hahah this is right" << endl;
	  			}
	  			min_dist = 9999;
	  		}
	  	}
	  }


  cout << "left index        : " << left_index.size() << endl;
  cout << "right index       : " << right_index.size() << endl;
  cout << "center index size : " << center_index.size() << endl;
  cout << "center size       : " << center.size() << endl;
  if(center.size() > 0){
    for(int i = 0; i < left_index.size();i++){
      outline_cones.push_back(center[left_index[i]]);
    }
    for(int i = 0; i < right_index.size();i++){
      inline_cones.push_back(center[right_index[i]]);
    }
  }
  sort(outline_cones.begin(),outline_cones.end(),track_cmp);
  sort(inline_cones.begin(),inline_cones.end(),track_cmp);
}

void Task1::make_line() {
  vector<double> input_x_l, input_y_l,input_x_r,input_y_r;
  input_x_l.clear();
  input_y_l.clear();
  input_x_r.clear();
  input_y_r.clear();
  double ds = 0.01;

  if(outline_cones.size() > 0){
      input_x_l = {-0.2};
      input_y_l = {first_l};
  }
  if(inline_cones.size() > 0){
      input_x_r = {-0.2};
      input_y_r = {first_r};
  }


  for (size_t i = 0; i < outline_cones.size(); i++) 
  {
    double x,y;

    x = outline_cones[i].x;
    y = outline_cones[i].y;
    input_x_l.push_back(x);
    input_y_l.push_back(y);

  }

  for (size_t i = 0; i < inline_cones.size(); i++) 
  {
    double x,y;
    x = inline_cones[i].x;
    y = inline_cones[i].y;
    
    input_x_r.push_back(x);
    input_y_r.push_back(y);
  }


  if (input_x_l.size() < 3){//예외처리 코드
    input_x_l.clear();
    input_y_l.clear();
    double before_dx, before_dy;
    if(input_x_r.size() >=3){
      for(int i = 0; i < input_x_r.size(); i++){
        if(i < input_x_r.size()-1){
          double dx = input_x_r[i+1] - input_x_r[i];
          before_dx = dx;
          double dy = input_y_r[i+1] - input_y_r[i];
          before_dy = dy;
          
          double m = -1 * dx/dy;
          if((dx > 0 && dy > 0) || (dx < 0 && dy > 0)){
            input_x_l.push_back(input_x_r[i] - (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_l.push_back(input_x_r[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }
        }
        else{
          double dx = before_dx;
          double dy = before_dy;
          
          double m = -1 * dx/dy;
          if((dx > 0 && dy > 0) || (dx < 0 && dy > 0)){
            input_x_l.push_back(input_x_r[i] - (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_l.push_back(input_x_r[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_l.push_back(input_y_r[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }
 
        }
      }
    }
    else{
      input_x_l.resize(2);
      input_y_l.resize(2);
      input_x_l = {0.0,0.6};
      input_y_l = {0.25,0.25};
    }
    calc_spline_course(input_x_l,input_y_l,l_rx,l_ry,l_ryaw,l_rk,ds);
  }
  else{
    calc_spline_course(input_x_l,input_y_l,l_rx,l_ry,l_ryaw,l_rk,ds);
  }  

  if (input_x_r.size() < 3){//예외처리 코드
    input_x_r.clear();
    input_y_r.clear();
    double before_dx,before_dy;
    if(input_x_l.size() >=3){
      for(int i = 0; i < input_x_l.size(); i++){
        if(i < input_x_l.size() - 1){
          double dx = input_x_l[i+1] - input_x_l[i];before_dx = dx;
          double dy = input_y_l[i+1] - input_y_l[i];before_dy = dy;
          
          double m = -1 * dx/dy;

          if((dx > 0 && dy < 0) || (dx < 0 && dy < 0)){
          input_x_r.push_back(input_x_l[i] - (dist_offset) / (sqrt(m*m + 1)));
          input_y_r.push_back(input_y_l[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_r.push_back(input_x_l[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_r.push_back(input_y_l[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }
        }
        else{
          double dx = before_dx;
          double dy = before_dy;
          
          double m = -1 * dx/dy;

          if((dx > 0 && dy < 0) || (dx < 0 && dy < 0)){
          input_x_r.push_back(input_x_l[i] - (dist_offset) / (sqrt(m*m + 1)));
          input_y_r.push_back(input_y_l[i] - (dist_offset)*m / (sqrt(m*m + 1)));
          }
          else{
            input_x_r.push_back(input_x_l[i] + (dist_offset) / (sqrt(m*m + 1)));
            input_y_r.push_back(input_y_l[i] + (dist_offset)*m / (sqrt(m*m + 1)));
          }

        }
      }
    }
    else{
      input_x_r.resize(2);
      input_y_r.resize(2);
      input_x_r = {0.0,0.6};
      input_y_r = {-0.25,-0.25};
    }
    calc_spline_course(input_x_r,input_y_r,r_rx,r_ry,r_ryaw,r_rk,ds);
  }
  else{
    calc_spline_course(input_x_r,input_y_r,r_rx,r_ry,r_ryaw,r_rk,ds);
  }

  nav_msgs::Path path_msg_l;
  nav_msgs::Path path_msg_r;

  path_msg_l.header.stamp = ros::Time::now();
  path_msg_l.header.frame_id = frame_id;
  for (size_t i = 0; i < l_rx.size(); i++)
  {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = l_rx[i];
      pose.pose.position.y = l_ry[i];
      pose.pose.orientation.w = 1.0;
      path_msg_l.poses.push_back(pose);
  }
  pub_path_l.publish(path_msg_l);

  path_msg_r.header.stamp = ros::Time::now();
  path_msg_r.header.frame_id = frame_id;
  for (size_t i = 0; i < r_rx.size(); i++)
  {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = r_rx[i];
      pose.pose.position.y = r_ry[i];
      pose.pose.orientation.w = 1.0;
      path_msg_r.poses.push_back(pose);
  }
  pub_path_r.publish(path_msg_r);


  if (r_ry.size() >= 3){
    first_r = 0;
    first_r += (r_ry[0] + r_ry[1] + r_ry[2]);
    first_r /= 3;
  }
  if (l_rx.size() >= 3){
    first_l = 0;
    first_l += (l_ry[0] + l_ry[1] + l_ry[2]);
    first_l /= 3;
  }
}

bool Task1::is_in_vector(vector<int> v, int element){
    vector<int>::iterator it;
    it = find(v.begin(), v.end(), element);
    if (it != v.end()) {
        return true;
    } else {
        return false;
    }
}

void Task1::make_waypoint(){    //size = (int)(rx.size()*0.5);
  vector<double> input_x, input_y,rx,ry,ryaw,rk;
  double ds = 0.01;
  float angle = 0.0f;
  float speed = 0.0f;
  int size_ = (l_rx.size() >= r_rx.size()) ? r_rx.size() : l_rx.size();
  int count = 0;
  double sum_x = 0, sum_y = 0;
  int size;

  //중앙경로 생성
  for (int i = 0; i < size_; i++){
    if(count%3 == 0){
      sum_x += (l_rx[i] + r_rx[i])/2;
      sum_y += (l_ry[i] + r_ry[i])/2;
      // 3 << "sum_x : : " << sum_x << endl;
      input_x.push_back(sum_x/3);
      input_y.push_back(sum_y/3);
      sum_x = 0;
      sum_y = 0;
    }
    else{
      sum_x += (l_rx[i] + r_rx[i])/2;
      sum_y += (l_ry[i] + r_ry[i])/2;
    }
    count++;
  }

  if(inline_cones.size() > 0 && outline_cones.size() >0){
    input_x.push_back((l_rx[l_rx.size()-1] + r_rx[r_rx.size()-1])/2);
    input_y.push_back((l_ry[l_ry.size()-1] + r_ry[r_ry.size()-1])/2);
  }

  input_x.erase(input_x.begin());
  input_y.erase(input_y.begin());

  if (input_x.size() < 2){//예외처리 코드
    input_x = {0.0,0.6};
    input_y = {0.0,0.0};
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
  }
  else{
    calc_spline_course(input_x,input_y,rx,ry,ryaw,rk,ds);
  }

  visualize_path(rx, ry, frame_id);

  size = (int)(rx.size()*0.5);

// --------------------------주행-------------------------------------------------------------------------//
  //cout << "waypoint size" << rx.size() << endl;

// ---------------flag----------------
  if(rx.size() <= done_size) {
    done_count++;
    if(done_count == done_count_param){
    track_done_flag.data = 4;
    pub_flag.publish(track_done_flag);
    cout << "~~~~~~~~~~~~~~~~ don't flag ~~~~~~~~~~~~~" <<endl;
    }
  }
  else done_count = 0;
// -----------------------------------

  geometry_msgs::PoseStamped target_p;
  if (rx.size() > 20)
  {
  //------------define angle---------
    //angle = pure_pursuit(rx, ry, target_p,speed);
    // angle = stanley(rx,ry,ryaw,rk);
    angle = pure_pursuit(rx, ry, target_p,speed,rk);
    cout << "angle: " << angle << endl;
    before_angle = GetAverage(avg_vec_);

    wp.x = target_p.pose.position.x;
    wp.y = target_p.pose.position.y;
  }
  else{
    // cout << "방어 steer" << endl;
    angle = before_angle;
  }

  if(sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2))<0.2){
    angle = before_angle;
  }

  //4.범위 이외시 최대조향각
  if (angle >= 0.2)
    angle = 0.2;
  if (angle <= -0.2)
    angle = -0.2;

  //speed = return_speed(angle,rx,ry,rk); 
  // speed_avg_vec_.push_back(speed);
  // if (speed_avg_vec_.size() > 5){speed_avg_vec_.erase(speed_avg_vec_.begin());}
  // speed_total_ = GetAverage(speed_avg_vec_);


  steer_total_ = angle;

  SteerAngle.data = steer_total_;

  //pub_speed.publish(Speed_Int32);
  pub_steering_angle.publish(SteerAngle);
  cout << "============ final ============" << endl;
  cout << "steer_total_  = " << SteerAngle.data << endl;
  cout << "rx.size= " << rx.size() <<endl;
  cout << "done_flag: " << track_done_flag.data << endl;
  cout << "\n" <<endl;

  avg_vec_.push_back(angle);
  
  if(avg_vec_.size() > 10){ avg_vec_.erase(avg_vec_.begin());}

}

int Task1::GetAverage(vector<double> const& vec){
    if (vec.empty()){
        return 0;
    }
    return accumulate(vec.begin(),vec.end(),0.0) / vec.size();
}

float Task1::pure_pursuit(vector<double>& rx, vector<double>& ry, geometry_msgs::PoseStamped& target_p,float& speed,vector<double>& rk)
{
  // 1. 목표 추종점 찾기
  //float lookahead_dist = speed* 0.2 + 0.5; // temp_speed = m/s
  // float lookahead_dist = 5.5;

  // #### LD 피팅 ####
// 1안
    // float lookahead_dist = 0.03*(0.03*current_speed_*current_speed_*current_speed_+7.5*current_speed_)+2;
// 2안
    // lookahead_dist_ = tanf(0.08*cur_speed_ - 0.09)+4.0;
    float lookahead_dist = 0.9;

    geometry_msgs::PoseStamped local_pose;
    local_pose.pose.position.x = 0.0;
    local_pose.pose.position.y = 0.0;    
    
    //int min_idx = find_cloestindex(local_pose,rx,ry);
    // double ratio_gain = 0;
    // if (rk.size() > 11) ratio_gain =abs(rk[10]);
    // else ratio_gain = abs(rk.back());
    // if(ratio_gain > 0.2) lookahead_dist = 4.5;

  float final_ld, alpha;
  float cur_x = -0.25f; // 휠베이스
  float cur_y = 0.0f;
  float dist = 0.0f;
  if(20 < rx.size() && rx.size() < 90) // 위에 Ld랑 맞추기
  {
    target_p.pose.position.x = rx[rx.size()-1];
    target_p.pose.position.y = ry[ry.size()-1];
    target_index_ = rx.size()-1;
    final_ld = sqrt(pow(target_p.pose.position.x-cur_x,2)+pow(target_p.pose.position.y-cur_y,2));
  }
  else
  {
    for (int i = 0; i < rx.size(); i++)
    {
      dist = sqrt(pow(rx[i]-cur_x,2) + pow(ry[i]-cur_y,2));
      if (dist > lookahead_dist)
      {
        target_index_ = i;
        final_ld = dist; // 최종으로 사용할 ld
        break;
      }
      if (i == rx.size())
      {
        // cout << "30개 이상일 때 방어 " << endl;
        target_index_ = i;
        final_ld = dist;
        break;
      }
    }
    target_p.pose.position.x = rx[target_index_];
    target_p.pose.position.y = ry[target_index_];
  }
  visualize_marker_geo(&target_p, frame_id, 1);
  double x = target_p.pose.position.x + abs(cur_x);
  double y = target_p.pose.position.y;

  // 2. alpha 구하기 
  alpha = -1*atan2f(y,x); // 일반적으로 아는 축으로 맞춤
  // if(alpha < -90.0 && alpha > -180.0f) alpha += 270.0f;   //지우는게 맞는듯...?
  // else alpha -= 90.0f;
  // alpha = -alpha*M_PI/180.0f;
  
  //3. 최종 조향각
  if (final_ld > 0.9f)
    final_ld = 0.9f;
  if (final_ld < 0.2f)
    final_ld = 0.2f;

  float cur_steer = atan2f(2.0f * 0.25f * sinf(alpha) / (final_ld), 1.0f) * 180.0f / M_PI ; 
  if(cur_steer>0) cur_steer *= -0.08;
  else if(cur_steer<0) cur_steer *= -0.08;

  cout << "===== Pure-pursuit ====="<< endl;
  cout << "target_index_" << target_index_<<endl;
  cout << "lookahead distance   : " << lookahead_dist <<endl;
  cout << "final ld             : " << final_ld <<endl;
  cout << "pure pursuit steer   : " << cur_steer << endl;
  cout << "dist: " << sqrt(pow(target_p.pose.position.x,2)+pow(target_p.pose.position.y,2)) <<endl;

  return cur_steer;

}

void Task1::run(){
  if(track_done_flag.data != 4){
    vector_clear();
    set_roi(msg_, roi);
    voxel(msg_, voxel_size);
    clustering(msg_, center, param);
    decision();
    //visualize_rviz(center, frame_id , 0);
    rviz_clear(frame_id);
    visualize_rviz(outline_cones, frame_id , 1);
    visualize_rviz(inline_cones, frame_id , 2);
    make_line();
    make_waypoint(); 
  }
}

