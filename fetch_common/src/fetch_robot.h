//
// Created by yz on 2020/6/7.
//

#ifndef FETCH_WS_FETCH_ROBOT_H
#define FETCH_WS_FETCH_ROBOT_H

#include <iostream>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
//
//#include <moveit_msgs/AttachedCollisionObject.h>

//
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/GripperCommandAction.h>
//#include <robot_calibration_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/Twist.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_srvs/SetBool.h"

#include <std_srvs/Trigger.h>
#include <stdlib.h>
#include "my_utility.h"
#include "env.h"

#define PI 3.1415926

class FetchRobot
{
public:
    FetchRobot(ros::NodeHandle& nodehandle, std::string move_group_name);

    void gripper(double position, double effort=0.0);

    void sendTargetPoseToTF(double x, double y, double z, double ox, double oy, double oz, double ow);

    geometry_msgs::TransformStamped getTargetTransform();

    bool goToPoseGoalWithTorso(double ox, double oy, double oz, double ow, double x, double y, double z);

    bool moveTorso(double distance);

    void addOrientationConstrainToEEF();
    void removeOrientationConstrainToEEF();

    void visualPlan();

    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);

    bool pick(double x, double y, double z, double roll=0, double pitch=0, double yaw=0);

    bool place(double x, double y, double z, double roll=0, double pitch=0, double yaw=0);

    bool detectObj(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res);

    bool detectPlane(std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res);

    bool pickObj(std_srvs::Trigger::Request &req,
                 std_srvs::Trigger::Response &res);

    bool placeObj(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res);
private:
    ros::NodeHandle nh_;

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head_action_client;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;

    ros::ServiceServer detect_obj_server;
    ros::ServiceServer detect_plane_server;
    ros::ServiceServer pick_obj_server;
    ros::ServiceServer place_obj_server;

    ros::ServiceClient nocs_client;

    ros::Publisher display_publisher;
//    ros::Publisher base_cmd_publisher;
//    ros::Publisher pub_aco;
    bool moveRealRobot;

    MyUtility myUtility;
    Env env;

    double obj_x, obj_y, obj_z;
    double obj_ox, obj_oy, obj_oz, obj_ow;
};

#endif //FETCH_WS_FETCH_ROBOT_H
