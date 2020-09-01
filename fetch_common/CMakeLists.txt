cmake_minimum_required(VERSION 3.0.2)
project(fetch_common)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    control_msgs
    actionlib_msgs
    actionlib
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    tf
    gazebo_msgs
)

generate_messages(
        DEPENDENCIES actionlib_msgs control_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fetch_common
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_server
        src/fetch_robot.h
        src/fetch_robot.cpp
        src/env.h
        src/env.cpp
        src/my_utility.h
        src/my_utility.cpp
        src/robot_server.cpp)
target_link_libraries(robot_server
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})
install(TARGETS robot_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})