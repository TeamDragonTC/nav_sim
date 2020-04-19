#include <nav_sim/nav_sim.h>

void NavSim::initialize() {
  currnet_pose_pub_ =
      pnh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

  cmd_vel_sub_ = pnh_.subscribe("/cmd_vel", 1, &NavSim::callback_cmd_vel, this);
  initialpose_sub_ =
      pnh_.subscribe("/initialpose", 1, &NavSim::callback_initialpose, this);
}

void NavSim::update_pose() {
  const double current_time = ros::Time::now().toSec();
  const double sampling_time = current_time - previous_time_;

  yaw_ += w_ * sampling_time;

  current_pose_.header.stamp = ros::Time::now();
  current_pose_.header.frame_id = "base_link";
  current_pose_.pose.position.x +=
      cmd_vel_.linear.x * std::cos(yaw_) * sampling_time;
  current_pose_.pose.position.y +=
      cmd_vel_.linear.y * std::sin(yaw_) * sampling_time;
  current_pose_.pose.position.z = 0.0;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw_);
  current_pose_.pose.orientation.w = quat.w();
  current_pose_.pose.orientation.x = quat.x();
  current_pose_.pose.orientation.y = quat.y();
  current_pose_.pose.orientation.z = quat.z();

  previous_time_ = current_time;
}

void NavSim::publish_state() {
  tf2_ros::TransformBroadcaster base_link_broadcaster;
  geometry_msgs::TransformStamped base_link_transform;

  base_link_transform.header.frame_id = "map";
  base_link_transform.child_frame_id = "base_link";
  base_link_transform.header.stamp = ros::Time::now();
  base_link_transform.transform.translation.x = current_pose_.pose.position.x;
  base_link_transform.transform.translation.y = current_pose_.pose.position.y;
  base_link_transform.transform.translation.z = current_pose_.pose.position.z;
  base_link_transform.transform.rotation.x = current_pose_.pose.orientation.x;
  base_link_transform.transform.rotation.y = current_pose_.pose.orientation.y;
  base_link_transform.transform.rotation.z = current_pose_.pose.orientation.z;
  base_link_transform.transform.rotation.w = current_pose_.pose.orientation.w;

  base_link_broadcaster.sendTransform(base_link_transform);
}

void NavSim::plan_velocity() {}