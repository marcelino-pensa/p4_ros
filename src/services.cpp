
#include "p4_ros/services.h"

namespace p4_ros {

bool minTimeService(p4_ros::min_time::Request  &req,
                    p4_ros::min_time::Response &res) {
	ROS_INFO("[p4_ros] Running minimum time solver!");

	// Problem variables
	std::vector<double> times;
	std::vector<p4::NodeEqualityBound> node_eq;
	std::vector<p4::SegmentInequalityBound> segment_ineq;
	p4::PolynomialSolver::Options solver_options;
	std::vector<p4::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// Setup optimization problem
	p4_helper::setup_min_time_problem(req, &times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	ros::Time t0 = ros::Time::now();
	p4::PolynomialSolver solver(solver_options);
	const p4::PolynomialPath path =
	    solver.Run(times, node_eq, node_ineq, segment_ineq);
	ros::Time t1 = ros::Time::now();

	// Best path
	p4::PolynomialPath path2;
	std::vector<double> times_final;
	p4_helper::solve_optimal_time_problem(times, node_eq, segment_ineq,
		                                  solver_options, node_ineq,
		                                  &times_final, &path2);
	ros::Time t2 = ros::Time::now();

	// Return structure
	const uint n_seg = times_final.size() - 1;
	for (uint idx = 0; idx < n_seg; idx++) {
		p4_ros::PolyPVA X, Y, Z;
		X = p4_helper::segment_pva_coeff_from_path(path2, times_final, idx, p4_helper::DimensionIdx::X);
		Y = p4_helper::segment_pva_coeff_from_path(path2, times_final, idx, p4_helper::DimensionIdx::Y);
		Z = p4_helper::segment_pva_coeff_from_path(path2, times_final, idx, p4_helper::DimensionIdx::Z);
		res.X.push_back(X);
		res.Y.push_back(Y);
		res.Z.push_back(Z);
	}

	// p4_helper::plot_results_3d(times, path);
	// p4_helper::plot_results_3d(times_final, path2);
	// p4_helper::plot_results(times_final, path2);
	// ROS_WARN("First solver time: %f", (t1-t0).toSec());
	// ROS_WARN("Second solver time: %f", (t2-t1).toSec());

	// std::cout << "Initial times: " << std::endl;
	// for (uint i = 0; i < times.size(); i++) {
	// 	std::cout << times[i] << "\t";
	// }
	// std::cout << std::endl;

	// std::cout << "Final times: " << std::endl;
	// for (uint i = 0; i < times_final.size(); i++) {
	// 	std::cout << times_final[i] << "\t";
	// }
	// std::cout << std::endl;
}

bool minAccXYService(p4_ros::minAccXYWpPVA::Request  &req,
                     p4_ros::minAccXYWpPVA::Response &res) {
	ROS_INFO("[p4_ros] Running minimum acceleration XY solver!");

	// Problem variables
	std::vector<double> times;
	std::vector<p4::NodeEqualityBound> node_eq;
	std::vector<p4::SegmentInequalityBound> segment_ineq;
	p4::PolynomialSolver::Options solver_options;
	std::vector<p4::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// Setup optimization problem
	p4_helper::setup_min_acc_problem(req, &times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	p4::PolynomialSolver solver(solver_options);
	const p4::PolynomialPath path =
	    solver.Run(times, node_eq, node_ineq, segment_ineq);

	// Return structure
	const uint n_seg = times.size() - 1;
	for (uint idx = 0; idx < n_seg; idx++) {
		p4_ros::PolyPVA X, Y;
		X = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::X);
		Y = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::Y);
		res.X.push_back(X);
		res.Y.push_back(Y);
	}
	// res.cost = path.optimal_cost;

	// p4_helper::plot_results(times, path);

	ROS_INFO("[p4_services] Returning request for %zu waypoints.", times.size());

	return true;
}

}  // namespace p4_ros