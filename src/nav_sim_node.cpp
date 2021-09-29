#include <nav_sim/nav_sim.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_sim_node");
  NavSim node;
  ros::spin();
  return 0;
}
