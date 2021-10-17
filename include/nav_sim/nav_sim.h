#ifndef _NAV_SIM_H_
#define _NAV_SIM_H_

#include <cmath>
#include <mutex>
#include <random>

#include <yaml-cpp/yaml.h>

#include <nav_sim/data_struct.hpp>
#include <nav_sim/noise.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_sim/LandmarkInfo.h>
#include <nav_sim/LandmarkInfoArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class NavSim
{
public:
  NavSim()
  {
    initialize();
  }
  ~NavSim()
  {
  }

  std::vector<Landmark> parseYaml(const std::string yaml);
  void initialize();
  void updateBasePose(const geometry_msgs::PoseWithCovarianceStamped pose_with_covariance, State& state);
  void publishTransform(const geometry_msgs::PoseStamped pose, const std::string child_frame_id);
  void observation(std::vector<Landmark> landmark_queue);
  State motion(const double vel, const double omega, const double dt, State pose);
  void decision(
    State& state, geometry_msgs::PoseStamped& pose, double v, double w, std::string frame_id, ros::Time stamp,
    double sampling_time, bool error);

  inline double normalizeDegree(const double degree)
  {
    double normalize_deg = std::fmod((degree + 180.0), 360.0) - 180.0;
    if (normalize_deg < -180.0)
      normalize_deg += 360.0;
    return normalize_deg;
  }

  inline void callbackCmdVel(const geometry_msgs::Twist& msg)
  {
    cmd_vel_ = msg;
  }
  void callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void timerCallback(const ros::TimerEvent& e);

  void clearMarker();

private:
  State current_state_;
  State ground_truth_;

  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{ "~" };
  ros::Timer timer_;
  ros::Time current_stamp_;

  std::shared_ptr<Noise> noise_ptr_;

  double period_;
  double limit_view_angle_;
  double error_coeff_;
  double previous_time_;

  std::string config_;

  // ランドマークの真値(world座標系)
  std::vector<Landmark> landmark_pose_list_;
  nav_sim::LandmarkInfo landmark_queue_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped ground_truth_pose_;

  ros::Publisher current_velocity_publisher_;
  ros::Publisher ground_truth_publisher_;
  ros::Publisher odometry_publisher_;
  ros::Publisher observation_publisher_;
  ros::Publisher landmark_info_pub_;
  ros::Publisher currnet_pose_publisher_;
  ros::Publisher path_pub_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber initialpose_sub_;
};

#endif
