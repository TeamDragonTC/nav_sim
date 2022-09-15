
cmake_minimum_required(VERSION 3.5)
project(nav_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(ndt_convergence_evaluation PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

ament_auto_add_executable(nav_sim_node
  src/nav_sim_node.cpp
  src/nav_sim_core.cpp
)

target_link_libraries(nav_sim_node yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_auto_package(INSTALL_TO_SHARE launch config rviz)
