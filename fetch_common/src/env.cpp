#include "env.h"

Env::Env(ros::NodeHandle& nodehandle):nh_(nodehandle){
    client = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
}

void Env::addCollisionObjects(double x, double y, double z)
{
    // Creating Environment
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Define the object that we will be manipulating
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.08;
    collision_objects[0].primitives[0].dimensions[1] = 0.08;
    collision_objects[0].primitives[0].dimensions[2] = 0.205;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = x;
    collision_objects[0].primitive_poses[0].position.y = y;
    collision_objects[0].primitive_poses[0].position.z = z;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

     /* Define the primitive and its dimensions. */
    collision_objects[1].id = "table1";
    collision_objects[1].header.frame_id = "object";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.05;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;
    collision_objects[1].primitives[0].dimensions[2] = 0.01;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = -0.5;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    collision_objects[1].operation = collision_objects[1].ADD;
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool Env::getTargetPose(gazebo_msgs::GetModelState& target,
                        const std::string& model_name,
                        const std::string& relative_entity_name){
    target.request.model_name = model_name;
    target.request.relative_entity_name = relative_entity_name;
    if (client.call(target)){
        return true;
    }
    else{
        ROS_WARN_STREAM("Failed to call service /gazebo/get_model_state");
        return false;
    }
}

void Env::addTable(double x, double y, double z, double width, double length, double height)
{
    moveit_msgs::CollisionObject placeTable;
    std::string tag_id_;
    if (nh_.getParam("tag_id", tag_id_)) {ROS_INFO("Got param: %s", tag_id_.c_str());}
    else {ROS_ERROR("Failed to get param 'tag_id'");}
    placeTable.header.frame_id = tag_id_;
    placeTable.id = "table2";

    /* Define the primitive and its dimensions. */
    placeTable.primitives.resize(1);
    placeTable.primitives[0].type = placeTable.primitives[0].BOX;
    placeTable.primitives[0].dimensions.resize(3);
    placeTable.primitives[0].dimensions[0] = width;
    placeTable.primitives[0].dimensions[1] = length;
    placeTable.primitives[0].dimensions[2] = height;

    placeTable.primitive_poses.resize(1);
    placeTable.primitive_poses[0].position.x = x;
    placeTable.primitive_poses[0].position.y = y;
    placeTable.primitive_poses[0].position.z = z;
    placeTable.primitive_poses[0].orientation.w = 1.0;

    placeTable.operation = placeTable.ADD;

    planning_scene_interface.applyCollisionObject(placeTable);
}

void Env::removeObject(const std::string object){
    std::vector<std::string> objs;
    objs.push_back(object);
    planning_scene_interface.removeCollisionObjects(objs);
}
