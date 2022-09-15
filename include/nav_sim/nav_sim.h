#ifndef _NAV_SIM_H_
#define _NAV_SIM_H_

#include <chrono>
#include <cmath>
#include <mutex>
#include <random>

#include <yaml-cpp/yaml.h>

#include <nav_sim/data_struct.hpp>
#include <nav_sim/noise.hpp>

#include <tf2/LinearMath/Quaternion.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_sim_msgs/msg/landmark_info.hpp>
#include <nav_sim_msgs/msg/landmark_info_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class NavSim : public rclcpp::Node
{
public:
  NavSim();
  ~NavSim() = default;

  std::vector<Landmark> parseYaml(const std::string yaml);
  void updateBasePose(
    const geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance, State & state);
  void publishTransform(
    const geometry_msgs::msg::PoseStamped pose, const std::string child_frame_id);
  void observation(std::vector<Landmark> landmark_queue);
  State motion(const double vel, const double omega, const double dt, State pose);
  void decision(
    State & state, geometry_msgs::msg::PoseStamped & pose, double v, double w, std::string frame_id,
    rclcpp::Time stamp, double sampling_time, bool error);

  inline double normalizeDegree(const double degree)
  {
    double normalize_deg = std::fmod((degree + 180.0), 360.0) - 180.0;
    if (normalize_deg < -180.0) normalize_deg += 360.0;
    return normalize_deg;
  }

  inline void callbackCmdVel(const geometry_msgs::msg::Twist msg) { cmd_vel_ = msg; }
  void callbackInitialpose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void timerCallback();

  void clearMarker();

private:
  State current_state_;
  State ground_truth_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time current_stamp_;

  std::shared_ptr<Noise> noise_ptr_;

  double period_;
  double limit_view_angle_;
  double error_coeff_;
  double previous_time_;

  std::string config_;

  // ランドマークの真値(world座標系)
  std::vector<Landmark> landmark_pose_list_;
  nav_sim_msgs::msg::LandmarkInfo landmark_queue_;

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped ground_truth_pose_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_sim_msgs::msg::LandmarkInfoArray>::SharedPtr observation_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_info_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscriber_;
};

#endif
