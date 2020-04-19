#include <nav_sim/nav_sim.h>

void NavSim::initialize() {
  pnh_.param<double>("error_coeff", error_coeff_, 0.01);

  currnet_pose_pub_ =
      pnh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

  cmd_vel_sub_ = pnh_.subscribe("/cmd_vel", 1, &NavSim::callback_cmd_vel, this);
  initialpose_sub_ =
      pnh_.subscribe("/initialpose", 1, &NavSim::callback_initialpose, this);
}

void NavSim::run() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(100);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(m_);
      update_pose();
    }
    rate.sleep();
  }
}

void NavSim::plan_velocity(double &target_v, double &target_w) {
  target_v = 1.0 * (cmd_vel_.linear.x - v_);
  target_w = 1.0 * (cmd_vel_.angular.z - w_);
}

void NavSim::sim_transfer_error(geometry_msgs::PoseStamped &pose) {}

void NavSim::update_pose() {
  const double current_time = ros::Time::now().toSec();
  const double sampling_time = current_time - previous_time_;

  double plan_v, plan_w;
  plan_velocity(plan_v, plan_w);

  yaw_ = yaw_ + w_ * sampling_time;

  current_pose_.header.stamp = ros::Time::now();
  current_pose_.header.frame_id = "base_link";
  current_pose_.pose.position.x =
      current_pose_.pose.position.x + v_ * std::cos(yaw_) * sampling_time;
  current_pose_.pose.position.y =
      current_pose_.pose.position.y + v_ * std::sin(yaw_) * sampling_time;
  current_pose_.pose.position.z = 0.0;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw_);
  current_pose_.pose.orientation.w = quat.w();
  current_pose_.pose.orientation.x = quat.x();
  current_pose_.pose.orientation.y = quat.y();
  current_pose_.pose.orientation.z = quat.z();

  v_ += (plan_v * sampling_time);
  w_ += (plan_w * sampling_time);

  // add normaol distribution error
  sim_transfer_error(current_pose_);
  // publish tf (covert pose stamped to transform stamped).
  publish_pose_to_transform(current_pose_, "base_link");
  // publish current pose;
  currnet_pose_pub_.publish(current_pose_);

  previous_time_ = current_time;
}

void NavSim::publish_pose_to_transform(geometry_msgs::PoseStamped pose,
                                       std::string frame) {
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
