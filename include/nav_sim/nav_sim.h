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

struct State {
  double x;
  double y;
  double yaw;
  State() : x(0.0), y(0.0), yaw(0.0) {}
};

class NavSim {
public:
  NavSim() : nh_(), pnh_("~"), v_(0.0), w_(0.0), { initialize(); }
  ~NavSim() {}

  void initialize();
  void run();
  void update_pose();
  void convert_to_pose(geometry_msgs::PoseStamped &pose, State state);
  void sim_transfer_error(State &state);
  void publish_pose_to_transform(geometry_msgs::PoseStamped pose,
                                 std::string frame);
  void plan_velocity(double &target_v, double &target_w);

  void callback_cmd_vel(const geometry_msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(m_);
    cmd_vel_ = msg;
  }
  void callback_initialpose(
      const geometry_msgs::PoseWithCovarianceStamped &msg) {
    std::lock_guard<std::mutex> lock(m_);
  }

private:
  State state_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  double error_coeff_;
  double previous_time_;
  double v_, w_;

  std::mutex m_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::PoseStamped current_pose_;

  ros::Publisher currnet_pose_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber initialpose_sub_;
};

#endif