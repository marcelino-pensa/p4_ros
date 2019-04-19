
#include "p4_ros/services.h"

namespace p4_ros {

bool minAccXYService(mg_msgs::minAccXYWpPVA::Request  &req,
                     mg_msgs::minAccXYWpPVA::Response &res) {
	// Problem variables
	std::vector<double> times;
	std::vector<mediation_layer::NodeEqualityBound> node_eq;
	std::vector<mediation_layer::SegmentInequalityBound> segment_ineq;
	mediation_layer::PolynomialSolver::Options solver_options;
	std::vector<mediation_layer::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// Setup optimization problem
	p4_helper::setup_problem(req, &times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	mediation_layer::PolynomialSolver solver(solver_options);
	const mediation_layer::PolynomialPath path =
	    solver.Run(times, node_eq, node_ineq, segment_ineq);

	// Return structure
	const uint n_seg = times.size() - 1;
	for (uint idx = 0; idx < n_seg; idx++) {
		mg_msgs::PolyPVA X, Y;
		X = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::X);
		Y = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::Y);
		res.X.push_back(X);
		res.Y.push_back(Y);
	}
	// res.cost = path.optimal_cost;

	ROS_INFO("[p4_services] Returning request for %zu waypoints.", times.size());
}

}  // namespace p4_ros