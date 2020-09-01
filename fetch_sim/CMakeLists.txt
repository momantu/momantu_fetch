cmake_minimum_required(VERSION 2.8.3)
project(fetch_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
        #  LIBRARIES simulation
        #  CATKIN_DEPENDS roscpp rospy tf tf2
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
