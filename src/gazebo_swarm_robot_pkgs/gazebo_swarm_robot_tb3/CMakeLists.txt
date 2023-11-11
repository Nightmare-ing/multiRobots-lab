################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_swarm_robot_tb3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS gazebo
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(gazebo_swarm_robot_control_angle src/main_angle.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(gazebo_swarm_robot_control_angle ${catkin_LIBRARIES})
add_executable(gazebo_swarm_robot_control_line src/main_line.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(gazebo_swarm_robot_control_line ${catkin_LIBRARIES})

add_executable(gazebo_stop_robot src/stop_robot.cpp src/swarm_robot_control.cpp include/swarm_robot_control.h)
target_link_libraries(gazebo_stop_robot ${catkin_LIBRARIES})




################################################################################
# Install
################################################################################


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

