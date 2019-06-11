#ifndef P4_SERVICES_H_
#define P4_SERVICES_H_

#include "ros/ros.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "p4_ros/time_optimizer_class.h"

#include "p4_ros/minAccXYWpPVA.h"
#include "p4_ros/min_time.h"
#include "p4_ros/p4_helper.h"

// Ros message types
#include "p4_ros/visualization_functions.h"

namespace p4_ros {

class ServicesClass {
 public:
	ServicesClass(ros::NodeHandle *nh);
	~ServicesClass() { };

	bool minAccXYService(p4_ros::minAccXYWpPVA::Request  &req,
                         p4_ros::minAccXYWpPVA::Response &res);

	bool minTimeService(p4_ros::min_time::Request  &req,
	                    p4_ros::min_time::Response &res);

 private:
 	ros::NodeHandle *nh_;
	ros::ServiceServer min_acc_srv, min_time_srv;
	visualization::TimeOptimizerPublisher time_optimizer_pub_obj;
};

}  // namespace p4_ros

#endif  // P4_SERVICES_H_