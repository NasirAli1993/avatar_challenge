cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(draw_shapes draw_shapes.cpp)

ament_target_dependencies(draw_shapes
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

target_include_directories(draw_shapes PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS
  draw_shapes
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
