#include <nav_sim/nav_sim.h>

void NavSim::initialize()
{
  pnh_.param<double>("error_coeff", error_coeff_, 0.01);
  pnh_.param<double>("limit_view_angle", limit_view_angle_, 45.0);
  pnh_.param<std::string>("config", config_, "");

  landmark_info_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("landmark_info", 1);
  currnet_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  path_pub_ = pnh_.advertise<nav_msgs::Path>("landmark_path", 1);

  cmd_vel_sub_ = pnh_.subscribe("/cmd_vel", 1, &NavSim::callbackCmdVel, this);
  initialpose_sub_ = pnh_.subscribe("/initialpose", 1, &NavSim::callbackInitialpose, this);

  landmark_pose_list_ = parseYaml(config_);

  timer_ = nh_.createTimer(ros::Duration(0.01), &NavSim::timerCallback, this);
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

void NavSim::timerCallback(const ros::TimerEvent & e)
{
  const double current_time = ros::Time::now().toSec();
  const double sampling_time = current_time - previous_time_;

  // calculate velocity using p control.
  double plan_v, plan_w;
  velocityFilter(plan_v, plan_w);

  // calculate next robot pose from target velocity
  state_.yaw_ += w_ * sampling_time;
  state_.x_ += v_ * std::cos(state_.yaw_) * sampling_time;
  state_.y_ += v_ * std::sin(state_.yaw_) * sampling_time;

  // add error by normal distribution
  simTransferError(state_);

  // convert State to geometry_msgs::PoseStamped
  current_pose_ = convertToPose<State>(state_);
  current_pose_.header.stamp = ros::Time::now();
  current_pose_.header.frame_id = "base_link";
  publishPoseToTransform(current_pose_, current_pose_.header.frame_id);

  // update velocity
  v_ += (plan_v * sampling_time);
  w_ += (plan_w * sampling_time);

  clearMarker();
  int landmark_id = 0;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped path_pose;
  visualization_msgs::MarkerArray landmark_info_array;
  visualization_msgs::Marker landmark_info;
  for (auto landmark : landmark_pose_list_) {
    const geometry_msgs::PoseStamped landmark_pose = convertToPose<Landmark>(landmark);

    const tf2::Transform map_to_base = convertToTransform(current_pose_);
    const tf2::Transform map_to_landmark = convertToTransform(landmark_pose);
    // base_link to landmark transform
    const tf2::Transform base_to_landmark = map_to_base.inverse() * map_to_landmark;
    // limit view angle for landmark detection
    double diff_landmark_yaw = std::atan2(
                                 map_to_landmark.getOrigin().y() - map_to_base.getOrigin().y(),
                                 map_to_landmark.getOrigin().x() - map_to_base.getOrigin().x()) -
                               state_.yaw_;
    const double diff_deg =
      normalizeDegree((diff_landmark_yaw * 180.0 / M_PI));  // normalize angle -180~180
    const double distance = std::sqrt(
      std::pow(base_to_landmark.getOrigin().x(), 2) +
      std::pow(base_to_landmark.getOrigin().y(), 2));
    if (diff_deg < limit_view_angle_ and -limit_view_angle_ < diff_deg) {
      path_pose.pose.position.x = 0.0;
      path_pose.pose.position.y = 0.0;
      path.poses.push_back(path_pose);
      path_pose.pose.position.x = base_to_landmark.getOrigin().x();
      path_pose.pose.position.y = base_to_landmark.getOrigin().y();
      path.poses.push_back(path_pose);

      landmark_info.header.frame_id = "base_link";
      landmark_info.header.stamp = current_pose_.header.stamp;
      landmark_info.text = "distance: " + std::to_string(distance) + " m \n" +
                           "yaw_diff: " + std::to_string(diff_deg) + " deg";
      landmark_info.pose.position.x = base_to_landmark.getOrigin().x() / 2.0;
      landmark_info.pose.position.y = base_to_landmark.getOrigin().y() / 2.0;
      landmark_info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
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
    path.header.frame_id = current_pose_.header.frame_id;
    path.header.stamp = current_pose_.header.stamp;

    publishPoseToTransform(landmark_pose, landmark.landmark_id_);
  }

  path_pub_.publish(path);
  landmark_info_pub_.publish(landmark_info_array);

  // publish current pose;
  currnet_pose_pub_.publish(current_pose_);

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
  state_.x_ = msg.pose.pose.position.x;
  state_.y_ = msg.pose.pose.position.y;
  tf2::Quaternion quat(
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  state_.yaw_ = yaw;
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

void NavSim::simTransferError(State & state)
{
  std::random_device seed;
  std::default_random_engine engine(seed());
  std::normal_distribution<> dist(0.0, 1.0);

  state.x_ += (error_coeff_)*dist(engine);
  state.y_ += (error_coeff_)*dist(engine);
  state.yaw_ += (error_coeff_)*dist(engine);
}

void NavSim::publishPoseToTransform(geometry_msgs::PoseStamped pose, std::string frame)
{
  static tf2_ros::TransformBroadcaster base_link_broadcaster;
  geometry_msgs::TransformStamped base_link_transform;

  base_link_transform.header.frame_id = "map";
  base_link_transform.child_frame_id = frame;
  base_link_transform.header.stamp = ros::Time::now();
  base_link_transform.transform.translation.x = pose.pose.position.x;
  base_link_transform.transform.translation.y = pose.pose.position.y;
  base_link_transform.transform.translation.z = pose.pose.position.z;
  base_link_transform.transform.rotation.x = pose.pose.orientation.x;
  base_link_transform.transform.rotation.y = pose.pose.orientation.y;
  base_link_transform.transform.rotation.z = pose.pose.orientation.z;
  base_link_transform.transform.rotation.w = pose.pose.orientation.w;

  base_link_broadcaster.sendTransform(base_link_transform);
}
