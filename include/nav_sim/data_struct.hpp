#ifndef _DATA_STRUCT_HPP_
#define _DATA_STRUCT_HPP_

#include <iostream>

struct State
{
  double x_;
  double y_;
  double yaw_;
  State() : x_(0.0), y_(0.0), yaw_(0.0)
  {
  }
  State(double x, double y, double yaw) : x_(x), y_(y), yaw_(yaw)
  {
  }
  State operator+(State pose_a)
  {
    State pose_b;
    pose_b.x_ = this->x_ + pose_a.x_;
    pose_b.y_ = this->y_ + pose_a.y_;
    pose_b.yaw_ = this->yaw_ + pose_a.yaw_;
    return pose_b;
  }
  State operator-(State pose_a)
  {
    State pose_b;
    pose_b.x_ = this->x_ - pose_a.x_;
    pose_b.y_ = this->y_ - pose_a.y_;
    pose_b.yaw_ = this->yaw_ - pose_a.yaw_;
    return pose_b;
  }
};

struct Landmark
{
  int landmark_id_;
  double x_;
  double y_;
  double yaw_;
  Landmark() : x_(0.0), y_(0.0), landmark_id_(-1)
  {
  }
  Landmark(double x, double y, int landmark_id) : x_(x), y_(y), landmark_id_(landmark_id)
  {
  }
};

#endif
