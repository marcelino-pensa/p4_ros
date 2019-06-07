
#ifndef TRAPEZOIDAL_HELPER_H_
#define TRAPEZOIDAL_HELPER_H_

#include "geometry_msgs/Point.h"
#include <Eigen/Dense>

namespace trapezoidal {

void time_from_displacements(
		const std::vector<double> &cumulative_displacement,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times);

void time_from_waypoints(
		const std::vector<geometry_msgs::Point> &waypoints,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times);

}  // namespace trapezoidal

#endif  // TRAPEZOIDAL_HELPER_H_
