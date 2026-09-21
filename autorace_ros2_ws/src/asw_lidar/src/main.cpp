// ------------main문 헤더 설명-----------------|
// 각각의 hpp파일은 Task1~4로 일단 구성            |
// config의 yaml파일은 task.yaml과 state.yaml   |
// task.yaml -> ROI param 등 값들 포함          |
// state.yaml -> 기존 parent.yaml파일로 스테이트  |
// :모라이때처럼 코드 작성하면 state.yaml은 없어도 됨 |
// Task1~4.hpp -> 정민이 코드 분석해서 우리식으로변환|
//--------------------------------------------|

#include <boost/shared_ptr.hpp>

//-----Task.hpp include-----|
#include "task/Task1.hpp" //꼬깔 적은 버전
#include "task/Task2.hpp" //터널
#include "task/Task3.hpp" //정적 회피
#include "task/Task4.hpp" //동적 회피
#include "task/Task5.hpp" //꼬깔 많은 버전
#include "task/Task6.hpp" //처음 아크릴 회피
//--------------------------|

//-----Int 선언--------------------------------------------------------| 
int camera_state, Task1_state, Task2_state, Task3_state, Task4_state, Task5_state, Task6_state;
//------------------------------------u--------------------------------|

//-----Task.hpp 단축키 설정------|
boost::shared_ptr<Task1> a;  //라바콘
boost::shared_ptr<Task2> b; 
boost::shared_ptr<Task3> c;  //로터리+차단기
boost::shared_ptr<Task4> d; 
boost::shared_ptr<Task5> e;  //터널
boost::shared_ptr<Task6> f;
//-----------------------------|

//---------Camera State pub Callback------------------|
void stateCallback(coss_msgs::msg::Coss state_) {
    camera_state = state_.mission_state;
    cout << "state: " << camera_state << endl;
}
//----------------------------------------------------|

//---------Vehicle Contrl pub Callback(vehi.ctrl. can't be used)----------|
// void stateCallback(const waypoint_maker::msg::State::ConstSharedPtr &state_msg) {
//     mission_state = state_msg->current_state;
//     cout << "mission_state: " << mission_state << endl;
// }
//------------------------------------------------------------------------|

void mission(){

    //-------------For Testing(cout)------------------|
    if(camera_state == 0) a->run();
    if(camera_state == 2) c->run();
    if(camera_state == 3) e->run();
    // c->run();
    //e->run();
    //-----------After State--------------------------|
    // if(camera_state == 0) d->run();
	// else if(camera_state == 5) s->run();
    // else if(camera_state == 9) t->run();
    // else if(camera_state == 7) g->run();
    // else if(camera_state == 4) f->run();
    // else cout<<"Not in lidar state!!!"<<endl;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "asw_lidar");
    ros::NodeHandle nh_;
    ros::Subscriber sub_state_ = nh_.subscribe("mission_state", 1, &stateCallback);

	//nh_.param("/state/mission_state", mission_state,-1);
	nh_.param("/state/Task1_state", Task1_state,-1);
    nh_.param("/state/Task2_state", Task2_state,-1);
    nh_.param("/state/Task3_state", Task3_state,-1);
	nh_.param("/state/Task4_state", Task4_state,-1);
    nh_.param("/state/Task5_state", Task5_state,-1);
    nh_.param("/state/Task6_state", Task6_state,-1);

    a.reset(new Task1());
    b.reset(new Task2());
    c.reset(new Task3());
	d.reset(new Task4());
    e.reset(new Task5());
    f.reset(new Task6());

    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        mission();
        loop_rate.sleep();
    }

    return 0;
}