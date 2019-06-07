
// ROS
#include "ros/ros.h"

// services
#include "p4_ros/services.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "p4_services");
	ros::NodeHandle node;

	ROS_INFO("[p4_services] Starting services!");
	ros::ServiceServer minAcc_Srv = node.advertiseService("minAccSolver", p4_ros::minAccXYService);
	ros::ServiceServer minTime_Srv = node.advertiseService("min_time_solver", p4_ros::minTimeService);

	ros::spin();

    return 0;
}