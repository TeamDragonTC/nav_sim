#ifndef _CONVERT_HPP_
#define _CONVERT_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

nav_msgs::msg::Odometry convertToOdometry(const geometry_msgs::msg::PoseStamped pose)
{
  nav_msgs::msg::Odometry odom;

  odom.pose.pose = pose.pose;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";

  return odom;
}

tf2::Transform convertToTransform(const geometry_msgs::msg::PoseStamped pose)
{
  tf2::Transform transform;
  transform.setOrigin(
    tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  transform.setRotation(tf2::Quaternion(
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
    pose.pose.orientation.w));
  return transform;
}

template <typename PoseType>
geometry_msgs::msg::PoseStamped convertToPose(PoseType state)
{
  geometry_msgs::msg::PoseStamped pose;
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

#endif
