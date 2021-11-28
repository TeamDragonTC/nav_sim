#include <nav_sim/nav_sim.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSim>());
  rclcpp::shutdown();
  return 0;
}
