#include <nav_sim/convert.hpp>

#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include <nav_sim/nav_sim.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

const double NOISE_PER_METER = 5.0;
const double NOISE_STD = M_PI / 60.0;

NavSim::NavSim() : Node("nav_sim")
{
  period_ = this->declare_parameter("period", 0.01);
  error_coeff_ = this->declare_parameter("error_coeff", 0.01);
  limit_view_angle_ = this->declare_parameter("limit_view_angle", 45.0);
  config_ = this->declare_parameter("config", "");

  double distance_noise_rate = this->declare_parameter("distance_noise_rate", 0.1);
  double direction_noise = this->declare_parameter("direction_noise", M_PI / 90.0);
  noise_ptr_ = std::make_shared<Noise>(distance_noise_rate, direction_noise);

  obstacle_radius_ = this->declare_parameter("obstacle_radius", 0.1);
  obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  current_velocity_publisher_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 10);
  ground_truth_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("ground_truth", 10);
  observation_publisher_ =
    this->create_publisher<nav_sim_msgs::msg::LandmarkInfoArray>("observation", 10);
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  landmark_info_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("landmark_info", 1);
  current_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("landmark_path", 10);
  obstacle_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_points", 10);

  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1, std::bind(&NavSim::callbackCmdVel, this, std::placeholders::_1));
  initialpose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1, std::bind(&NavSim::callbackInitialpose, this, std::placeholders::_1));
  initialpose_obstacle_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose_obstacle", 1,
      std::bind(&NavSim::callbackInitialPoseObstacleInfo, this, std::placeholders::_1));

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  landmark_pose_list_ = parseYaml(config_);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 * period_)),
    std::bind(&NavSim::timerCallback, this));

  current_stamp_ = rclcpp::Clock().now();
  previous_time_ = current_stamp_.seconds();
}

std::vector<Landmark> NavSim::parseYaml(const std::string yaml)
{
  YAML::Node config = YAML::LoadFile(yaml);

  int id = -1;
  std::vector<Landmark> landmark_pose_list;
  for (YAML::const_iterator itr = config.begin(); itr != config.end(); ++itr) {
    Landmark landmark;
    landmark.landmark_id_ = ++id;
    landmark.x_ = itr->second["x"].as<double>();
    landmark.y_ = itr->second["y"].as<double>();
    landmark_pose_list.push_back(landmark);
  }
  return landmark_pose_list;
}

void NavSim::observation(std::vector<Landmark> landmark_queue)
{
  clearMarker();

  int landmark_id = 0;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped path_pose;
  visualization_msgs::msg::MarkerArray landmark_info_array;
  visualization_msgs::msg::Marker landmark_info;
  nav_sim_msgs::msg::LandmarkInfo observation_result;
  nav_sim_msgs::msg::LandmarkInfoArray observation_result_array;

  const auto current_time_stamp = rclcpp::Clock().now();
  for (auto landmark : landmark_queue) {
    geometry_msgs::msg::PoseStamped landmark_pose = convertToPose<Landmark>(landmark);

    const tf2::Transform map_to_base = convertToTransform(current_pose_);
    const tf2::Transform map_to_landmark = convertToTransform(landmark_pose);
    // base_link to landmark transform
    const tf2::Transform base_to_landmark = map_to_base.inverse() * map_to_landmark;
    // limit view angle for landmark detection
    double diff_landmark_yaw = std::atan2(
                                 map_to_landmark.getOrigin().y() - map_to_base.getOrigin().y(),
                                 map_to_landmark.getOrigin().x() - map_to_base.getOrigin().x()) -
                               current_state_.yaw_;
    // ランドマークの位置を極座標系で計算する
    const double diff_deg =
      normalizeDegree((diff_landmark_yaw * 180.0 / M_PI));  // normalize angle -180~180
    const double distance = std::sqrt(
      std::pow(base_to_landmark.getOrigin().x(), 2) +
      std::pow(base_to_landmark.getOrigin().y(), 2));
    // 観測情報に対して雑音を乗せる(雑音->バイアス)
    const auto result = noise_ptr_->observationBias(
      noise_ptr_->observationNoise(std::make_pair(distance, diff_deg * M_PI / 180.0)));
    // 極座標から直交座標に変換する(可視化のため)
    const double base_to_landmark_x_with_noise = result.first * std::cos(result.second);
    const double base_to_landmark_y_with_noise = result.first * std::sin(result.second);

    if (diff_deg < limit_view_angle_ && -limit_view_angle_ < diff_deg) {
      observation_result.length = result.first;
      observation_result.theta = result.second;
      observation_result.id = landmark.landmark_id_;
      observation_result_array.landmark_array.push_back(observation_result);

      path_pose.pose.position.x = 0.0;
      path_pose.pose.position.y = 0.0;
      path.poses.push_back(path_pose);
      path_pose.pose.position.x = base_to_landmark_x_with_noise;
      path_pose.pose.position.y = base_to_landmark_y_with_noise;
      path.poses.push_back(path_pose);

      landmark_info.header.frame_id = "base_link";
      landmark_info.header.stamp = current_time_stamp;
      landmark_info.text = "distance:" + std::to_string(result.first) + "m\n" +
                           "yaw_diff:" + std::to_string(result.second * M_PI * 180.0) + "deg";
      landmark_info.pose.position.x = base_to_landmark.getOrigin().x() / 2.0;
      landmark_info.pose.position.y = base_to_landmark.getOrigin().y() / 2.0;
      landmark_info.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      landmark_info.id = landmark_id++;
      landmark_info.scale.x = 0.1;
      landmark_info.scale.y = 0.1;
      landmark_info.scale.z = 0.1;
      landmark_info.color.a = 1.0;
      landmark_info.color.r = 0.6;
      landmark_info.color.g = 0.8;
      landmark_info.color.b = 1.0;
      landmark_info_array.markers.push_back(landmark_info);
    }
    path.header.frame_id = "base_link";
    path.header.stamp = current_time_stamp;

    landmark_pose.header.frame_id = std::to_string(landmark.landmark_id_);
    publishTransform(landmark_pose, std::to_string(landmark.landmark_id_));
  }
  path_publisher_->publish(path);
  landmark_info_publisher_->publish(landmark_info_array);

  observation_result_array.header.stamp = current_time_stamp;
  observation_publisher_->publish(observation_result_array);
}

State NavSim::motion(const double vel, const double omega, const double dt, State pose)
{
  State diff_pose;
  const double t0 = pose.yaw_;

  diff_pose.yaw_ = omega * dt;
  if (std::fabs(omega) < 1e-06) {
    diff_pose.x_ = vel * std::cos(t0) * dt;
    diff_pose.y_ = vel * std::sin(t0) * dt;
  } else {
    diff_pose.x_ = (vel / omega) * (std::sin(t0 + omega * dt) - std::sin(t0));
    diff_pose.y_ = (vel / omega) * (-std::cos(t0 + omega * dt) + std::cos(t0));
  }

  return pose + diff_pose;
}

void NavSim::decision(
  State & state, geometry_msgs::msg::PoseStamped & pose, double v, double w, std::string frame_id,
  rclcpp::Time stamp, double sampling_time, bool error)
{
  state = motion(v, w, sampling_time, state);

  if (error) noise_ptr_->noise(state, cmd_vel_, sampling_time);

  pose = convertToPose<State>(state);
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  publishTransform(pose, frame_id);
}

void NavSim::timerCallback()
{
  current_stamp_ = rclcpp::Clock().now();
  const double current_time = current_stamp_.seconds();
  const double sampling_time = current_time - previous_time_;

  // move as ground truth
  decision(
    ground_truth_, ground_truth_pose_, cmd_vel_.linear.x, cmd_vel_.angular.z, "ground_truth",
    current_stamp_, sampling_time, false);
  // move as current pose with error
  decision(
    current_state_, current_pose_, noise_ptr_->bias(cmd_vel_.linear.x),
    noise_ptr_->bias(cmd_vel_.angular.z), "base_link", current_stamp_, sampling_time, true);

  // publish current velocity
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = current_stamp_;
  twist.header.frame_id = "base_link";
  twist.twist = cmd_vel_;
  current_velocity_publisher_->publish(twist);

  // observation landmark
  observation(landmark_pose_list_);

  // publish ground truth / current pose;
  current_pose_publisher_->publish(current_pose_);
  ground_truth_publisher_->publish(ground_truth_pose_);

  // publish obstacle cloud
  if (!obstacle_cloud_->points.empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tf2::Transform map2obstacle, base2obstacle, base2map;
    transformPointCloud(
      obstacle_cloud_, transformed_cloud, getTransform("base_link", "map", current_stamp_));
    sensor_msgs::msg::PointCloud2 obstacle_points_msg;
    pcl::toROSMsg(*transformed_cloud, obstacle_points_msg);
    obstacle_points_msg.header.stamp = current_stamp_;
    obstacle_points_msg.header.frame_id = "base_link";
    obstacle_cloud_publisher_->publish(obstacle_points_msg);
  }
  // publish odometry
  nav_msgs::msg::Odometry odom = convertToOdometry(current_pose_);
  odom.header.stamp = current_stamp_;
  odom.twist.twist = cmd_vel_;
  odometry_publisher_->publish(odom);

  previous_time_ = current_time;
}

void NavSim::clearMarker()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = "base_link";
  clear_marker.header.stamp = rclcpp::Clock().now();
  clear_marker.ns = "";
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_marker.pose.orientation.w = 1.0;
  markers.markers.push_back(clear_marker);
  landmark_info_publisher_->publish(markers);
}

void NavSim::updateBasePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance, State & state)
{
  state.x_ = pose_with_covariance.pose.pose.position.x;
  state.y_ = pose_with_covariance.pose.pose.position.y;
  tf2::Quaternion quat(
    pose_with_covariance.pose.pose.orientation.x, pose_with_covariance.pose.pose.orientation.y,
    pose_with_covariance.pose.pose.orientation.z, pose_with_covariance.pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  state.yaw_ = yaw;
}

void NavSim::callbackInitialpose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  updateBasePose(msg, current_state_);
  updateBasePose(msg, ground_truth_);
}

void NavSim::createObstacleCloud(
  const geometry_msgs::msg::Pose pose, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_ptr)
{
  cloud_ptr->header.frame_id = "map";
  const int res = obstacle_radius_ * 60;
  for (int j = 0; j < res; j++) {
    pcl::PointXYZ point;
    const double radian = 2.0 * M_PI * j / res;
    point.x = obstacle_radius_ * std::cos(radian) + pose.position.x;
    point.y = obstacle_radius_ * std::sin(radian) + pose.position.y;
    point.z = pose.position.z;
    cloud_ptr->points.emplace_back(point);
  }
}

void NavSim::callbackInitialPoseObstacleInfo(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  createObstacleCloud(msg.pose.pose, obstacle_cloud_ptr);

  // TODO: ray tracing
  *obstacle_cloud_ += *obstacle_cloud_ptr;
}

void NavSim::publishTransform(
  const geometry_msgs::msg::PoseStamped pose, const std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped base_link_transform;

  base_link_transform.header.frame_id = "map";
  base_link_transform.child_frame_id = child_frame_id;
  base_link_transform.header.stamp = pose.header.stamp;
  base_link_transform.transform.translation.x = pose.pose.position.x;
  base_link_transform.transform.translation.y = pose.pose.position.y;
  base_link_transform.transform.translation.z = pose.pose.position.z;
  base_link_transform.transform.rotation.x = pose.pose.orientation.x;
  base_link_transform.transform.rotation.y = pose.pose.orientation.y;
  base_link_transform.transform.rotation.z = pose.pose.orientation.z;
  base_link_transform.transform.rotation.w = pose.pose.orientation.w;

  broadcaster_->sendTransform(base_link_transform);
}

geometry_msgs::msg::TransformStamped NavSim::getTransform(
  const std::string target_frame, const std::string source_frame, const rclcpp::Time stamp)
{
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform =
      tf_buffer_.lookupTransform(target_frame, source_frame, stamp, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return frame_transform;
}

void NavSim::transformPointCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr & output_ptr,
  const geometry_msgs::msg::TransformStamped frame_transform)
{
  const Eigen::Affine3d frame_affine = tf2::transformToEigen(frame_transform);
  const Eigen::Matrix4f frame_matrix = frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_ptr, *output_ptr, frame_matrix);
}
