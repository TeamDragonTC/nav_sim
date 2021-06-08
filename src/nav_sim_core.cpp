#include <nav_sim/nav_sim.h>

const double NOISE_PER_METER = 5.0;
const double NOISE_STD = M_PI / 60.0;

void NavSim::initialize()
{
  pnh_.param<double>("distance_noise_rate", distance_noise_rate_, 0.1);
  pnh_.param<double>("direction_noise", direction_noise_, M_PI / 90);
  pnh_.param<double>("period", period_, 0.01);
  pnh_.param<double>("error_coeff", error_coeff_, 0.01);
  pnh_.param<double>("limit_view_angle", limit_view_angle_, 45.0);
  pnh_.param<std::string>("config", config_, "");

  // 移動に対して発生する雑音
  distance_until_noise_ = getExponentialDistribution(1.0 / 5.0);
  // 速度に対するバイアス誤差
  bias_rate_v_ = getGaussDistribution(1.0, 0.1);
  bias_rate_w_ = getGaussDistribution(1.0, 0.1);
  // スタック
  time_until_escape_ = getExponentialDistribution(1.0 / 60.0);
  time_until_stuck_ = getExponentialDistribution(1.0 / 60.0);
  // センサ値に対するバイアス
  distance_noise_std_ = getGaussDistribution(0.0, distance_noise_rate_);
  direction_noise_std_ = getGaussDistribution(0.0, direction_noise_);

  current_velocity_publisher_ = pnh_.advertise<geometry_msgs::TwistStamped>("twist", 10);
  ground_truth_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>("ground_truth", 10);
  observation_publisher_ = pnh_.advertise<nav_sim::LandmarkInfoArray>("observation", 10);
  odometry_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odom", 10);

  landmark_info_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("landmark_info", 1);
  currnet_pose_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
  path_pub_ = pnh_.advertise<nav_msgs::Path>("landmark_path", 10);

  cmd_vel_sub_ = pnh_.subscribe("/cmd_vel", 1, &NavSim::callbackCmdVel, this);
  initialpose_sub_ = pnh_.subscribe("/initialpose", 1, &NavSim::callbackInitialpose, this);

  landmark_pose_list_ = parseYaml(config_);

  timer_ = nh_.createTimer(ros::Duration(period_), &NavSim::timerCallback, this);

  current_stamp_ = ros::Time::now();
  previous_time_ = current_stamp_.toSec();
}

std::vector<Landmark> NavSim::parseYaml(const std::string yaml)
{
  YAML::Node config = YAML::LoadFile(yaml);

  std::vector<Landmark> landmark_pose_list;
  for (YAML::const_iterator itr = config.begin(); itr != config.end(); ++itr) {
    Landmark landmark;
    landmark.landmark_id_ = itr->first.as<std::string>();
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
  nav_msgs::Path path;
  geometry_msgs::PoseStamped path_pose;
  visualization_msgs::MarkerArray landmark_info_array;
  visualization_msgs::Marker landmark_info;
  nav_sim::LandmarkInfo observation_result;
  nav_sim::LandmarkInfoArray observation_result_array;

  const auto current_time_stamp = ros::Time::now();
  for (auto landmark : landmark_pose_list_) {
    geometry_msgs::PoseStamped landmark_pose = convertToPose<Landmark>(landmark);

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
    const auto result = observationBias(observationNoise(std::make_pair(distance, diff_deg)));
    // 極座標から直交座標に変換する(可視化のため)
    const double base_to_landmark_x_with_noise =
      result.first * std::cos(result.second * M_PI / 180.0);
    const double base_to_landmark_y_with_noise =
      result.first * std::sin(result.second * M_PI / 180.0);

    if (diff_deg < limit_view_angle_ && -limit_view_angle_ < diff_deg) {
      observation_result.length = result.first;
      observation_result.theta = result.second;
      observation_result.id = landmark_id++;
      observation_result_array.landmark_array.push_back(observation_result);

      path_pose.pose.position.x = 0.0;
      path_pose.pose.position.y = 0.0;
      path.poses.push_back(path_pose);
      path_pose.pose.position.x = base_to_landmark_x_with_noise;
      path_pose.pose.position.y = base_to_landmark_y_with_noise;
      path.poses.push_back(path_pose);

      landmark_info.header.frame_id = "base_link";
      landmark_info.header.stamp = current_time_stamp;
      landmark_info.text = "distance: " + std::to_string(result.first) + " m \n" +
                           "yaw_diff: " + std::to_string(result.second) + " deg";
      landmark_info.pose.position.x = base_to_landmark.getOrigin().x() / 2.0;
      landmark_info.pose.position.y = base_to_landmark.getOrigin().y() / 2.0;
      landmark_info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      landmark_info.id = landmark_id;
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

    landmark_pose.header.frame_id = landmark.landmark_id_;
    publishPoseToTransform(landmark_pose);
  }
  path_pub_.publish(path);
  landmark_info_pub_.publish(landmark_info_array);

  observation_result_array.header.stamp = current_time_stamp;
  observation_publisher_.publish(observation_result_array);
}

void NavSim::decision(
  State & state, geometry_msgs::PoseStamped & pose, double v, double w, std::string frame_id,
  ros::Time stamp, double sampling_time, bool error)
{
#if 0
  if(error) stuck(v, w, sampling_time);
#endif

  state.yaw_ += w * sampling_time;
  state.x_ += v * std::cos(state.yaw_) * sampling_time;
  state.y_ += v * std::sin(state.yaw_) * sampling_time;

  if (error) noise(state, sampling_time);

  pose = convertToPose<State>(state);
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  publishPoseToTransform(pose);
}

void NavSim::timerCallback(const ros::TimerEvent & e)
{
  current_stamp_ = ros::Time::now();
  const double current_time = current_stamp_.toSec();
  const double sampling_time = current_time - previous_time_;

  // calculate velocity using p control.
  double plan_v, plan_w;
  velocityFilter(plan_v, plan_w);

  // move as ground truth
  decision(
    ground_truth_, ground_truth_pose_, v_, w_, "ground_truth", current_stamp_, sampling_time,
    false);
  // move as current pose with error
  decision(
    current_state_, current_pose_, bias(v_, bias_rate_v_), bias(w_, bias_rate_w_), "base_link",
    current_stamp_, sampling_time, true);

  // update velocity
  v_ += (plan_v * sampling_time);
  w_ += (plan_w * sampling_time);

  // observation landmark
  observation(landmark_pose_list_);

  // publish ground truth / current pose;
  currnet_pose_publisher_.publish(current_pose_);
  ground_truth_publisher_.publish(ground_truth_pose_);

  previous_time_ = current_time;
}

void NavSim::clearMarker()
{
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = "base_link";
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "";
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker.pose.orientation.w = 1.0;
  markers.markers.push_back(clear_marker);
  landmark_info_pub_.publish(markers);
}

void NavSim::velocityFilter(double & target_v, double & target_w)
{
  target_v = 1.0 * (cmd_vel_.linear.x - v_);
  target_w = 1.0 * (cmd_vel_.angular.z - w_);
}

void NavSim::callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  current_state_.x_ = msg.pose.pose.position.x;
  current_state_.y_ = msg.pose.pose.position.y;
  tf2::Quaternion quat(
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  current_state_.yaw_ = yaw;
}

template <typename PoseType>
geometry_msgs::PoseStamped NavSim::convertToPose(PoseType state)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = state.x_;
  pose.pose.position.y = state.y_;
  pose.pose.position.z = 0.0;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, state.yaw_);
  pose.pose.orientation.w = quat.w();
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  return pose;
}

tf2::Transform NavSim::convertToTransform(geometry_msgs::PoseStamped pose)
{
  tf2::Transform transform;
  transform.setOrigin(
    tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  transform.setRotation(tf2::Quaternion(
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
    pose.pose.orientation.w));
  return transform;
}

// スタックのシミュレーション
void NavSim::stuck(double & velocity, double & omega, double time_interval)
{
  if (is_stuck_) {
    time_until_escape_ -= time_interval;
    if (time_until_escape_ <= 0.0) {
      time_until_escape_ += getExponentialDistribution(1.0 / 60.0);
      is_stuck_ = false;
      return;
    }
    velocity = 0.0;
    omega = 0.0;
  } else {
    time_until_stuck_ -= time_interval;
    if (time_until_stuck_ <= 0.0) {
      time_until_stuck_ += getExponentialDistribution(1.0 / 60.0);
      is_stuck_ = true;
    }
  }
}

// 観測情報に対する雑音
std::pair<double, double> NavSim::observationNoise(const std::pair<double, double> position)
{
  const auto distance = position.first;
  const auto degree = position.second;

  const double gauss_for_distance = getGaussDistribution(distance, distance * distance_noise_rate_);
  const double gauss_for_degree = getGaussDistribution(degree, direction_noise_);

  return std::make_pair(gauss_for_distance, gauss_for_degree);
}

// 観測情報に対するバイアス誤差
std::pair<double, double> NavSim::observationBias(const std::pair<double, double> position)
{
  const auto distance = position.first;
  const auto degree = position.second;

  return std::make_pair(distance + distance * distance_noise_std_, degree + direction_noise_std_);
}

// 移動に対する雑音
void NavSim::noise(State & state, double time_interval)
{
  distance_until_noise_ = distance_until_noise_ - (std::fabs(cmd_vel_.linear.x) * time_interval +
                                                   std::fabs(cmd_vel_.angular.z) * time_interval);
  if (distance_until_noise_ <= 0.0) {
    distance_until_noise_ += getExponentialDistribution(1.0 / 5.0);
    state.yaw_ += getGaussDistribution(0.0, M_PI / 60.0);
  }
}

void NavSim::publishPoseToTransform(geometry_msgs::PoseStamped pose)
{
  static tf2_ros::TransformBroadcaster base_link_broadcaster;
  geometry_msgs::TransformStamped base_link_transform;

  base_link_transform.header.frame_id = "map";
  base_link_transform.child_frame_id = pose.header.frame_id;
  base_link_transform.header.stamp = pose.header.stamp;
  base_link_transform.transform.translation.x = pose.pose.position.x;
  base_link_transform.transform.translation.y = pose.pose.position.y;
  base_link_transform.transform.translation.z = pose.pose.position.z;
  base_link_transform.transform.rotation.x = pose.pose.orientation.x;
  base_link_transform.transform.rotation.y = pose.pose.orientation.y;
  base_link_transform.transform.rotation.z = pose.pose.orientation.z;
  base_link_transform.transform.rotation.w = pose.pose.orientation.w;

  base_link_broadcaster.sendTransform(base_link_transform);
}
