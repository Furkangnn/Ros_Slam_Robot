cmake_minimum_required(VERSION 3.8)
project(ros_slam_robot)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(robot_kinematics src/kinematics.cpp)
target_compile_features(robot_kinematics PUBLIC cxx_std_17)
target_include_directories(robot_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(serial_base src/serial_base.cpp)
target_compile_features(serial_base PUBLIC cxx_std_17)
target_link_libraries(serial_base robot_kinematics)
ament_target_dependencies(serial_base
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS robot_kinematics serial_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/fake_base.py
  scripts/odom_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics robot_kinematics)
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_kinematics)
ament_package()
