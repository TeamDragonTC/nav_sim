#ifndef _NAV_SIM_H_
#define _NAV_SIM_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class NavSim {
public:
  NavSim() : nh_(), pnh_("~") { initialize(); }
  ~NavSim();

  void initialize();

  void update_pose(geometry_msgs::Pose pose);
  void calc_velocity();
  void publish_state();

  void callback_cmd_vel(const geometry_msgs::Twist &msg) { cmd_vel_ = msg; }
  void callback_initialpose(
      const geometry_msgs::PoseWithCovarianceStamped &msg) {}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::PoseStamped current_pose_;

  ros::Publisher currnet_pose_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber initialpose_sub_;
};

#endif