cmake_minimum_required(VERSION 3.0.2)
project(go1_ros_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  unitree_legged_msgs
  unitree_legged_sdk
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES go1_ros_interface
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${unitree_legged_sdk_INCLUDE_DIRS}
)


add_executable(publish_robot_state src/publish_robot_state.cpp)
target_link_libraries(publish_robot_state ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(publish_robot_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})