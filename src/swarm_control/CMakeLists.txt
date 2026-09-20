cmake_minimum_required(VERSION 3.8)
project(swarm_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(single_drone_controller_node
  src/single_drone_controller_node.cpp
)
ament_target_dependencies(single_drone_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
)
target_link_libraries(single_drone_controller_node Eigen3::Eigen)

add_executable(mean_field_controller_node
  src/mean_field_controller_node.cpp
)
ament_target_dependencies(mean_field_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
)

add_executable(consensus_controller_node
  src/consensus_controller_node.cpp
)
ament_target_dependencies(consensus_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
)
target_link_libraries(consensus_controller_node Eigen3::Eigen)

# distributed_density_controller_node.cpp is intentionally not built, see
# its header comment. Its whole body is wrapped in #if 0 anyway.

install(TARGETS
  single_drone_controller_node
  mean_field_controller_node
  consensus_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
