

#include "p4_ros/trapezoidal.h"

namespace trapezoidal {

void time_from_displacements(
		const std::vector<double> &cumulative_displacement,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times) {
	// Time to get to max_vel from zero velocity
	const double t_ud = max_vel/max_acc;

	const uint n_points = cumulative_displacement.size();
	const double p0 = cumulative_displacement[0];
	const double pf = cumulative_displacement[n_points-1];
	
	// Find the final time based on displacement, max vel, max acc
	// Also, set some important constants
	double tf, A1, A2, A3;
	double v_top = max_vel;
	bool saturates;
	if (pf > t_ud*max_vel) {  // In case the velocity saturates at max_vel
		tf = t_ud + pf/max_vel;
		A1 = t_ud*max_vel/2.0;
		A2 = (tf-2.0*t_ud)*max_vel;
		A3 = A1;
		saturates = true;
	} else { // If velocity does not saturate at max_vel
		tf = 2.0*sqrt(pf/max_acc);
		v_top = sqrt(pf*max_acc);
		A1 = v_top*tf/4.0;
		A2 = A1;
		saturates = false;
	}

	// For each position in the displacement, calculate time to reach them
	std::vector<double> t;
	for (uint i = 0; i < n_points; i++) {
		const double p = cumulative_displacement[i];

		if (saturates) {
			if (p <= 0) {
				t.push_back(0.0);
			} else if (p < A1) {
				t.push_back(sqrt(2.0*p/max_acc));
			} else if (p < (A1 + A2)) {
				const double Pnew = p - A1;
				const double t2 = Pnew/max_vel;
				t.push_back(t_ud + t2);
			} else if (p < (A1 + A2 + A3)) {
				const double Pnew = p - A1 - A2;
				const double vf = sqrt(max_vel*max_vel - 2.0*max_acc*Pnew);
				const double t3 = 2.0*Pnew/(max_vel + vf);
				t.push_back(tf - t_ud + t3);
			} else {
				t.push_back(tf);
			}
		}

		if (!saturates) {
			if (p <= 0) {
				t.push_back(0.0);
			} else if (p < A1) {
				t.push_back(sqrt(2*p/max_acc));
			} else if (p < (A1 + A2)) {
				const double Pnew = p - A1;
        		const double vf = sqrt(v_top*v_top - 2.0*max_acc*Pnew);
        		const double t2 = 2.0*Pnew/(v_top + vf);
        		t.push_back(0.5*tf + t2);
			} else {
				t.push_back(tf);
			}
		}
	}

	*times = t;
}

void time_from_waypoints(
		const std::vector<geometry_msgs::Point> &waypoints,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times) {
	// Get number of points
	const uint n_points = waypoints.size();

	// Compute cumulative distance along waypoints
	std::vector<double> cumulative_displacement;
	cumulative_displacement.resize(n_points);
	cumulative_displacement[0] = 0.0;
	for (uint i = 1; i < n_points; i++) {
		const Eigen::Vector3d p1(waypoints[i-1].x, waypoints[i-1].y, waypoints[i-1].z);
		const Eigen::Vector3d p2(waypoints[i].x, waypoints[i].y, waypoints[i].z);
		const double dist = (p2 - p1).norm();
		cumulative_displacement[i] = cumulative_displacement[i-1] + dist;
	}

	time_from_displacements(cumulative_displacement, max_vel, max_acc, times);
}

}  // namespace trapezoidal