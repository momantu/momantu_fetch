#include "my_utility.h"

MyUtility::MyUtility(ros::NodeHandle& nodehandle):nh_(nodehandle){
    if (nh_.getParam("base_frame", base_frame_)) {ROS_INFO("Got param: %s", base_frame_.c_str());}
    else {ROS_ERROR("Failed to get param 'base_frame'");}

    if (nh_.getParam("eef_frame", eef_frame_)) {ROS_INFO("Got param: %s", eef_frame_.c_str());}
    else {ROS_ERROR("Failed to get param 'eef_frame'");}

    if (nh_.getParam("eef_parent_frame", eef_parent_frame_)) {ROS_INFO("Got param: %s", eef_parent_frame_.c_str());}
    else {ROS_ERROR("Failed to get param 'eef_parent_frame'");}

    std::string tf_error;
    ros::Time last_error = ros::Time::now();

    while (ros::ok() && !tf_.waitForTransform(base_frame_, eef_frame_, ros::Time(), ros::Duration(0.1), \
         ros::Duration(0.01), &tf_error)) {
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now()) {
            last_error = ros::Time::now();
        }
        tf_error.clear();
    }
}

bool MyUtility::GetGripperPose(tf::Transform& TS, std::string obj_frame_){
    bool flag = true;
    tf::StampedTransform transform_base_obj;
    try {
        tf_.lookupTransform( base_frame_, obj_frame_,  //the transform from frame /object_predicted to frame /base_link.
                                  ros::Time(0), transform_base_obj);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        flag = false;
    }

    tf::StampedTransform transform_wrist_gri;
    try{
        tf_.lookupTransform(eef_parent_frame_, eef_frame_,  //the transform from gripper frame to wrist frame (0.16645,0,0)
                                 ros::Time(0), transform_wrist_gri);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        flag = false;
    }
    if (flag == true){
        TS = multi_transforms(transform_base_obj, transform_wrist_gri);
        return true;
    }
    else{return false;}
}


bool MyUtility::GetPlacePosition(tf::Transform& TS, std::string obj_frame_){
    bool flag = true;
    tf::StampedTransform transform_base_obj;
    try {
        tf_.lookupTransform( base_frame_, obj_frame_,  //the transform from frame /object_predicted to frame /base_link.
                                  ros::Time(0), transform_base_obj);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        flag = false;
    }

    tf::Transform coordinate = transform_base_obj;

    tf::Quaternion q(0, 0, 0, 1);
    tf::Vector3 v(0, 0, 0.2);
    tf::Transform relative_pos(q, v);

    TS = coordinate * relative_pos;
    return true;
}

bool MyUtility::GetObjPose(tf::Transform& TS, std::string obj_frame_){
    bool flag = true;
    tf::StampedTransform transform_base_obj;
    try {
        tf_.lookupTransform( base_frame_, obj_frame_,  //the transform from frame /object_predicted to frame /base_link.
                             ros::Time(0), transform_base_obj);
        TS = transform_base_obj;
        ros::Duration(3.0).sleep();
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        flag = false;
    }
    return flag;
}

tf::Transform MyUtility::multi_transforms(tf::StampedTransform T1, tf::StampedTransform T2)
{
    tf::Transform T2i = (T2.inverse());
    tf::Transform T_res;
    T_res = T2i*T1;
    tf::StampedTransform TS;
    TS.setData(T_res);

    tf::Quaternion q1= T_res.getRotation();
    tf::Vector3 v1 = T1.getOrigin();
    tf::Vector3 vr = T_res.getOrigin();
    tf::Matrix3x3 M1;
    M1.setRotation(q1);
    tf::Vector3 v6,v7,v8;
    v6=M1[0];
    v7=M1[1];
    v8=M1[2];

    std::cout<<"translation1:"<<v1[0]<<","<<v1[1]<<","<<v1[2]<<std::endl;
    std::cout<<"translationr:"<<vr[0]<<","<<vr[1]<<","<<vr[2]<<std::endl;
    std::cout<<"rotation:"<<v6[0]<<","<<v6[1]<<","<<v6[2]<<std::endl;
    std::cout<<"        "<<v7[0]<<","<<v7[1]<<","<<v7[2]<<std::endl;
    std::cout<<"        "<<v8[0]<<","<<v8[1]<<","<<v8[2]<<std::endl;
    std::cout<<std::endl;
    ros::WallDuration(1.0).sleep();
    return T_res;
}

MyUtility::~MyUtility(){};
