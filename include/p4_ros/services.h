#ifndef P4_SERVICES_H_
#define P4_SERVICES_H_

#include "ros/ros.h"
#include "polynomial_solver.h"
#include "mg_msgs/minAccXYWpPVA.h"
#include "p4_ros/p4_helper.h"

namespace p4_ros {

bool minAccXYService(mg_msgs::minAccXYWpPVA::Request  &req,
                     mg_msgs::minAccXYWpPVA::Response &res);

}  // namespace p4_ros

#endif  // P4_SERVICES_H_