#ifndef _NAV_SIM_H_
#define _NAV_SIM_H_

#include <nav_sim/data_struct.hpp>
#include <nav_sim/noise.hpp>

#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <random>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_sim_msgs/msg/landmark_info.hpp>
#include <nav_sim_msgs/msg/landmark_info_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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

  geometry_msgs::msg::TransformStamped getTransform(
    const std::string target_frame, const std::string source_frame, const rclcpp::Time stamp);
  void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr & output_ptr,
    const geometry_msgs::msg::TransformStamped frame_transform);

  void createObstacleCloud(
    const geometry_msgs::msg::Pose pose, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_ptr);

  inline void callbackCmdVel(const geometry_msgs::msg::Twist msg) { cmd_vel_ = msg; }
  void callbackInitialpose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void callbackInitialPoseObstacleInfo(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
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

  double obstacle_radius_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_obstacle_subscriber_;
};

#endif
