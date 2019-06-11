
// ROS
#include "ros/ros.h"

// services
#include "p4_ros/services.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "p4_services");
	ros::NodeHandle node("~");

	ROS_INFO("[p4_services] Starting services!");
	p4_ros::ServicesClass services(&node);

	ros::spin();

    return 0;
}