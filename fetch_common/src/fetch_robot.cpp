#include "fetch_robot.h"


FetchRobot::FetchRobot(ros::NodeHandle& nodehandle, std::string move_group_name) :
        nh_(nodehandle), env(nodehandle), myUtility(nodehandle),
        head_action_client("/head_controller/point_head",true),
        gripper_action_client("/gripper_controller/gripper_action",true),
        move_group(move_group_name)
{
    move_group.setPlannerId("RRTkConfigDefault");
    nh_.param("move_real_robot", moveRealRobot, true);
    ROS_INFO("Waiting for gripper action server to start.");
    gripper_action_client.waitForServer();
    ROS_INFO("Gripper action server started, sending goal.");
    ROS_INFO("Waiting for head action server to start.");
    head_action_client.waitForServer();
    ROS_INFO("Head action server started, sending goal.");
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    detect_obj_server = nh_.advertiseService("/detect_obj_srv", &FetchRobot::detectObj,this);
    detect_plane_server = nh_.advertiseService("/detect_plane_srv", &FetchRobot::detectPlane,this);
    pick_obj_server = nh_.advertiseService("/pick_srv", &FetchRobot::pickObj,this);
    place_obj_server = nh_.advertiseService("/place_srv", &FetchRobot::placeObj,this);

    nocs_client = nh_.serviceClient<std_srvs::SetBool>("/estimate_pose_nocs");
//    move_group.setPlannerId("RRTConnectkConfigDefault");

}

bool FetchRobot::detectObj(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res){
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (nocs_client.call(srv)){
        ROS_INFO("enable NOCS successfully");
    }
    else{
        ROS_ERROR("Failed to call service estimate_pose_nocs");
        return false;
    }

    ros::Duration(1).sleep();

    tf::Transform TS;
    while (!myUtility.GetObjPose(TS, "/object_predicted")) {
        ROS_WARN_STREAM("waiting to obtain obj pose");
    }
    env.addCollisionObjects(TS.getOrigin().x(),
                            TS.getOrigin().y(),
                            TS.getOrigin().z() + 0.05);
    ROS_INFO_STREAM("add obj as collision obj");
    ros::Duration(1).sleep();

    while (!myUtility.GetGripperPose(TS, "/object_predicted")) {
        ROS_WARN_STREAM("waiting to obtain gripper pose");
    }
    ROS_INFO ( "detectObj" );

    res.success = true;
    return true;
}

bool FetchRobot::detectPlane(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res){
    tf::Transform TS;
    std::string tag_id_;
    if (nh_.getParam("tag_id", tag_id_)) {ROS_INFO("Got param: %s", tag_id_.c_str());}
    else {ROS_ERROR("Failed to get param 'tag_id'");}
    while (!myUtility.GetObjPose(TS, tag_id_)) {
        ROS_WARN_STREAM("waiting to apriltag pose");
    }
    ROS_INFO ( "detectPlane") ;
    env.addTable();
    res.success = true;
    return true;
}

bool FetchRobot::pickObj(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res){
    tf::Transform TS;
    while (!myUtility.GetGripperPose(TS, "/object_predicted")) {
        ROS_WARN_STREAM("waiting to obtain obj pose");
    }
    ROS_INFO ("pickObj") ;
    obj_x  = TS.getOrigin().x();
    obj_y  = TS.getOrigin().y();
    obj_z  = TS.getOrigin().z() + 0.02;
    obj_ox = TS.getRotation().x();
    obj_oy = TS.getRotation().y();
    obj_oz = TS.getRotation().z();
    obj_ow = TS.getRotation().w();
    res.success = pick( obj_x, obj_y, obj_z, 0, 0, 0);

    addOrientationConstrainToEEF();

    if (res.success){
        std_srvs::SetBool srv;
        srv.request.data = false;
        if (nocs_client.call(srv)){
            ROS_INFO("disable NOCS successfully");
        }
        else{
            ROS_ERROR("Failed to call service estimate_pose_nocs");
            return false;
        }
    }
    return true;
}

void FetchRobot::addOrientationConstrainToEEF(){
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "wrist_roll_link";
    ocm.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    ocm.orientation = tf2::toMsg(orientation);
    ocm.absolute_x_axis_tolerance = PI / 3;
    ocm.absolute_y_axis_tolerance = PI / 3;
    ocm.absolute_z_axis_tolerance = PI;
    ocm.weight = 1;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);
}

void FetchRobot::removeOrientationConstrainToEEF(){
    move_group.clearPathConstraints();
}

bool FetchRobot::placeObj(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res){
    //addOrientationConstrainToEEF();

    tf::Transform TS;
    std::string tag_id_;
    if (nh_.getParam("tag_id", tag_id_)) {ROS_INFO("Got param: %s", tag_id_.c_str());}
    else {ROS_ERROR("Failed to get param 'tag_id'");}
    while (!myUtility.GetPlacePosition(TS, tag_id_)) {
        ROS_WARN_STREAM("waiting to obtain apriltag pose");
    }
    obj_x  = TS.getOrigin().x();
    obj_y  = TS.getOrigin().y();
    obj_z  = TS.getOrigin().z();
    obj_ox = TS.getRotation().x();
    obj_oy = TS.getRotation().y();
    obj_oz = TS.getRotation().z();
    obj_ow = TS.getRotation().w();
    ROS_INFO ( "placeObj" );
    ROS_INFO_STREAM("x: " << obj_x << " y: " << obj_y << " z: " << obj_z);
    ROS_INFO_STREAM("ox: " << obj_ox << " oy: " << obj_oy << " oz: " << obj_oz << " ow: " << obj_ow);
    res.success = place(obj_x, obj_y, obj_z, 0, 0, 0);

    removeOrientationConstrainToEEF();
    return true;
}

void FetchRobot::gripper(double position, double effort) {
    // range: [0, 0.1]
    // torso's height is around 0.4m
    control_msgs::GripperCommandGoal grasp_pos;
    grasp_pos.command.position = position;
    grasp_pos.command.max_effort = effort;
    gripper_action_client.sendGoal(grasp_pos);
}

bool FetchRobot::goToPoseGoalWithTorso(double ox, double oy, double oz, double ow, double x, double y, double z)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = ox;
    target_pose.orientation.y = oy;
    target_pose.orientation.z = oz;
    target_pose.orientation.w = ow;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group.setPoseTarget(target_pose);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success && moveRealRobot)
    {
        visualPlan();
        move_group.move();
    }
    std::cout<<"success: "<< success<< std::endl;
    return success;
}


bool FetchRobot::moveTorso(double distance)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(target_pose);
    target_pose.position.z += distance;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    move_group.setPlanningTime(10.0);

    double fraction = move_group.computeCartesianPath(waypoints,
                                                                0.01,  // eef_step
                                                                0.0,   // jump_threshold
                                                                trajectory,
                                                                true);
    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "arm_with_torso");
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    // Finally plan and execute the trajectory
    my_plan.trajectory_ = trajectory;
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
    if (success && moveRealRobot){
        move_group.execute(my_plan);
    }
}

void FetchRobot::visualPlan()
{
    for (int i = 0; i < 2; ++i) {
        ROS_INFO("Visualizing plan 1 (again)");
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(3.0);
    }
}

void FetchRobot::openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.1;
    posture.points[0].positions[1] = 0.1;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void FetchRobot::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.02;
    posture.points[0].positions[1] = 0.02;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

bool FetchRobot::pick(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting pre-grasp approach
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = x;
    grasps[0].grasp_pose.pose.position.y = y;
    grasps[0].grasp_pose.pose.position.z = z;

    // Setting posture of eef before grasp
    openGripper(grasps[0].pre_grasp_posture);

    // Setting post-grasp retreat
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.2;

    // Setting posture of eef during grasp
    closedGripper(grasps[0].grasp_posture);
    // Set support surface as table1.
    // For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    bool success = (move_group.pick("object", grasps) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // END_SUB_TUTORIAL
    std::cout << "pick end!!!" << std::endl;

    if (success){
        env.removeObject("table1");
    }
    return success;
}

bool FetchRobot::place(double x, double y, double z, double roll, double pitch, double yaw)
{
    std::cout << "place start!!!" << std::endl;
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in verbose mode." This is a known issue.
    // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
    // a single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = x;
    place_location[0].place_pose.pose.position.y = y;
    place_location[0].place_pose.pose.position.z = z;

    // Setting pre-place approach
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.098;

    // Setting post-grasp retreat
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    /* Similar to the pick case */

    // Set support surface as table2.
    // For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface.
    move_group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    bool success = (move_group.place("object", place_location) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "place end!!!" << std::endl;
    return success;
}

void FetchRobot::sendTargetPoseToTF(double x, double y, double z, double ox, double oy, double oz, double ow){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "head_camera_rgb_optical_frame";
    transformStamped.child_frame_id = "obj";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation.x = ox;
    transformStamped.transform.rotation.y = oy;
    transformStamped.transform.rotation.z = oz;
    transformStamped.transform.rotation.w = ow;
    br.sendTransform(transformStamped);
}


geometry_msgs::TransformStamped FetchRobot::getTargetTransform() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (1) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base_link", "obj",
                                                        ros::Time(0));
            return transformStamped;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
}




