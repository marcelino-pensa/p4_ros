
// ROS
#include "ros/ros.h"

// services
#include "p4_ros/services.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "~");
  ros::NodeHandle node;

  ROS_INFO("[p4_services] Starting services!");
  ros::ServiceServer minSnap_Srv = node.advertiseService("minAccSolver", p4_ros::minAccXYService);

  ros::spin();

  return 0;
}