// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN_ACKERMANN_TO_VESC_H_
#define VESC_ACKERMANN_ACKERMANN_TO_VESC_H_

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

namespace vesc_ackermann
{

class AckermannToVesc
{
public:

  AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  bool auto_control = false;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  ros::Publisher erpm_pub_;
  ros::Publisher servo_pub_;
  ros::Subscriber ackermann_sub_;
  ros::Subscriber auto_ackermann_sub_;
  ros::Subscriber is_auto_sub_;
  //ros::Subscriber ackermann_lane_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr& cmd);
  void autoackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr& cmd);
  void AutoCallback(const std_msgs::Bool::ConstSharedPtr& auto_msg_);
  //void ackermannlaneCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstSharedPtr& cmd);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_ACKERMANN_TO_VESC_H_
