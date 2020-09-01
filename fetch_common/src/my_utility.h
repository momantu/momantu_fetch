#ifndef FETCH_WS_MY_UTILITY_H
#define FETCH_WS_MY_UTILITY_H

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <string>
#include <ros/console.h>
class MyUtility {
public:
    MyUtility(ros::NodeHandle& nodehandle);
    ~MyUtility();
    bool GetGripperPose(tf::Transform& TS, std::string obj_frame_);
    bool GetObjPose(tf::Transform& TS, std::string obj_frame_);
    tf::Transform multi_transforms(tf::StampedTransform T1, tf::StampedTransform T2);
    bool GetPlacePosition(tf::Transform& TS, std::string obj_frame_);
protected:
    ros::NodeHandle nh_;
    tf::TransformListener tf_;
    std::string base_frame_, eef_frame_, eef_parent_frame_;
};


#endif //FETCH_WS_MY_UTILITY_H
