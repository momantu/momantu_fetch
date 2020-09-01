#ifndef FETCH_WS_ENV_H
#define FETCH_WS_ENV_H

#include <iostream>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gazebo_msgs/GetModelState.h>
#include <string>

class Env
{
public:
    Env(ros::NodeHandle& nodehandle);

    void addCollisionObjects(double x, double y, double z);
    bool getTargetPose(gazebo_msgs::GetModelState& target,
                        const std::string& model_name,
                        const std::string& relative_entity_name);
    void addTable(double x=0, double y=0, double z=-0.5, double width=0.05, double length=0.05, double height=0.01);
    void removeObject(const std::string object);

private:
    ros::NodeHandle nh_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::ServiceClient client;
};


#endif //FETCH_WS_ENV_H
