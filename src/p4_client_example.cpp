
// ROS
#include "ros/ros.h"

#include "p4_ros/minAccXYWpPVA.h"
#include "p4_ros/min_time.h"
#include "p4_ros/p4_helper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "p4_client");
    ros::NodeHandle node("~");

    // ros::spin();
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    ros::ServiceClient client = node.serviceClient<p4_ros::min_time>("/p4_services/min_time_solver");
    p4_ros::min_time req;
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 1.0));
    req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 1.5));
    req.request.pos_array.push_back(p4_helper::ros_point(0.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(1.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(2.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(3.0, 0.0, 2.0));
    req.request.pos_array.push_back(p4_helper::ros_point(4.0, 0.0, 2.0));

    req.request.sampling_freq = 30;
    req.request.corridor_width = 0.1;
    req.request.max_vel = 0.6;
    req.request.max_acc = 1.0;
    req.request.max_jerk = 2.5;
    req.request.visualize_output = true;

    ROS_INFO("[p4_services] Calling service %s!", client.getService().c_str());
    client.call(req);
    
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

  return 0;
}