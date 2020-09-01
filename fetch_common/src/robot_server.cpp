#include "fetch_robot.h"
#include "env.h"
#include "my_utility.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis_server_node");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error) ) {
       ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::WallDuration(1.0).sleep();

    std::string move_group_name_;
    if (node_handle.getParam("move_group_name", move_group_name_)) {ROS_INFO("Got param: %s", move_group_name_.c_str());}
    else {ROS_ERROR("Failed to get param 'move_group_name'");}
    FetchRobot fetchRobot(node_handle, move_group_name_);

   ros::waitForShutdown();

	return 0;
}

