#ifndef _NAV_SIM_H_
#define _NAV_SIM_H_

#include <mutex>
#include <random>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

struct State
{
  double x_;
  double y_;
  double yaw_;
  State() : x_(0.0), y_(0.0), yaw_(0.0) {}
  State(double x, double y, double yaw) : x_(x), y_(y), yaw_(yaw) {}
};

class NavSim
{
public:
  NavSim() : nh_(), pnh_("~"), v_(0.0), w_(0.0) { initialize(); }
  ~NavSim() {}

  void initialize();
  void convertToPose(geometry_msgs::PoseStamped & pose, State state);
  void simTransferError(State & state);
  void publishPoseToTransform(geometry_msgs::PoseStamped pose, std::string frame);
  void planVelocity(double & target_v, double & target_w);

  void callbackCmdVel(const geometry_msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(m_);
    cmd_vel_ = msg;
  }
  void callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped & msg);
  void timerCallback(const ros::TimerEvent & e);

private:
  State state_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer timer_;

  double error_coeff_;
  double previous_time_;
  double v_, w_;

  std::mutex m_;

  std::vector<State> landmark_pose_list_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::PoseStamped current_pose_;

  ros::Publisher currnet_pose_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber initialpose_sub_;
};

#endif
