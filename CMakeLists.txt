cmake_minimum_required(VERSION 2.8.3)
project(nav_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_message_files(
  FILES
  LandmarkInfo.msg
  LandmarkInfoArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #CATKIN_DEPENDS roscpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nav_sim_node src/nav_sim_core.cpp src/nav_sim_node.cpp)
target_link_libraries(nav_sim_node ${catkin_LIBRARIES} yaml-cpp)
