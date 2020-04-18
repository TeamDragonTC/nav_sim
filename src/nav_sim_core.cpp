#include <nav_sim/nav_sim.h>

void NavSim::initialize() {
  currnet_pose_pub_ =
      pnh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

  cmd_vel_sub_ = pnh_.subscribe("/cmd_vel", 1, &NavSim::callback_cmd_vel, this);
  initialpose_sub_ =
      pnh_.subscribe("/initialpose", 1, &NavSim::callback_initialpose, this);
}

void NavSim::update_pose(geometry_msgs::Pose pose) {}

void NavSim::calc_velocity() {}

void NavSim::publish_state() {}