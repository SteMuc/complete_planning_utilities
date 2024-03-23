cmake_minimum_required(VERSION 3.0.2)
project(complete_planning_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  complete_planning_msgs
  geometry_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  tf
  tf_conversions
  eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES complete_planning_pkg
 CATKIN_DEPENDS complete_planning_msgs geometry_msgs moveit_core moveit_ros_perception moveit_ros_planning_interface roscpp rospy sensor_msgs std_msgs std_srvs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

## Add your action server source files
add_executable(planning_server src/CartesianPlan.cpp src/JointPlan.cpp src/SlerpPlan.cpp src/PlanningServer.cpp)
add_executable(planning_client src/PlanningClient.cpp)

target_link_libraries(planning_server
  ${catkin_LIBRARIES}
)

target_link_libraries(planning_client
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Install the executable
install(TARGETS planning_server planning_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(planning_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} complete_planning_msgs_generate_messages_cpp)
add_dependencies(planning_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} complete_planning_msgs_generate_messages_cpp)
