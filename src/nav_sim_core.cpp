#include <nav_sim/nav_sim.h>

void NavSim::initialize()
{
  pnh_.param<double>("error_coeff", error_coeff_, 0.01);

  currnet_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

  cmd_vel_sub_ = pnh_.subscribe("/cmd_vel", 1, &NavSim::callbackCmdVel, this);
  initialpose_sub_ = pnh_.subscribe("/initialpose", 1, &NavSim::callbackInitialpose, this);

  timer_ = nh_.createTimer(ros::Duration(0.01), &NavSim::timerCallback, this);
}

void NavSim::timerCallback(const ros::TimerEvent& e)
{

  const double current_time = ros::Time::now().toSec();
  const double sampling_time = current_time - previous_time_;

  // calculate velocity using p control.
  double plan_v, plan_w;
  planVelocity(plan_v, plan_w);

  // calculate next robot pose from target velocity
  state_.yaw += w_ * sampling_time;
  state_.x += v_ * std::cos(state_.yaw) * sampling_time;
  state_.y += v_ * std::sin(state_.yaw) * sampling_time;

  // add error by normal distribution
  simTransferError(state_);

  // convert State to geometry_msgs::PoseStamped
  current_pose_.header.stamp = ros::Time::now();
  current_pose_.header.frame_id = "base_link";
  convertToPose(current_pose_, state_);

  // update velocity
  v_ += (plan_v * sampling_time);
  w_ += (plan_w * sampling_time);

  // publish tf (covert pose stamped to transform stamped).
  publishPoseToTransform(current_pose_, "base_link");
  // publish current pose;
  currnet_pose_pub_.publish(current_pose_);

  previous_time_ = current_time;
}

void NavSim::planVelocity(double & target_v, double & target_w)
{
  target_v = 1.0 * (cmd_vel_.linear.x - v_);
  target_w = 1.0 * (cmd_vel_.angular.z - w_);
}

void NavSim::callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::mutex> lock(m_);
  state_.x = msg.pose.pose.position.x;
  state_.y = msg.pose.pose.position.y;
  tf2::Quaternion quat(
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  state_.yaw = yaw;
}

void NavSim::convertToPose(geometry_msgs::PoseStamped & pose, State state)
{
  pose.pose.position.x = state.x;
  pose.pose.position.y = state.y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, state.yaw);
  pose.pose.orientation.w = quat.w();
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
}

void NavSim::simTransferError(State & state)
{
  std::random_device seed;
  std::default_random_engine engine(seed());
  std::normal_distribution<> dist(0.0, 1.0);

  state.x += (error_coeff_)*dist(engine);
  state.y += (error_coeff_)*dist(engine);
  state.yaw += (error_coeff_)*dist(engine);
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
