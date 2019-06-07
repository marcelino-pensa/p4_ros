#ifndef P4_SERVICES_H_
#define P4_SERVICES_H_

#include "ros/ros.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "p4_ros/minAccXYWpPVA.h"
#include "p4_ros/min_time.h"
#include "p4_ros/p4_helper.h"

namespace p4_ros {

bool minAccXYService(p4_ros::minAccXYWpPVA::Request  &req,
                     p4_ros::minAccXYWpPVA::Response &res);

bool minTimeService(p4_ros::min_time::Request  &req,
                    p4_ros::min_time::Response &res);

}  // namespace p4_ros

#endif  // P4_SERVICES_H_