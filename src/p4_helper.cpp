
#include "p4_ros/p4_helper.h"

namespace p4_helper {

std::vector<float> eigen_to_stdvector(const Eigen::VectorXd &eig_vec) {
	std::vector<float> vec;
	for (uint i = 0; i < eig_vec.size(); i++) {
		vec.push_back(eig_vec(i));
	}
	return vec;
}

std::vector<float> diff_coeff(const std::vector<float> &coeff_in) {
	uint coeff_size = coeff_in.size();
	std::vector<float> coeff_out;
	for (uint i = 0; i < coeff_size-1; i++) {
		coeff_out.push_back(coeff_in[i+1]);
	}
	coeff_out.push_back(0.0);

	return coeff_out;
}

void set_pos_eq_constraint(const geometry_msgs::Point &pos, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints) {
	p4::NodeEqualityBound eq_x(DimensionIdx::X, node_idx, DerivativeIdx::Position, pos.x);
	p4::NodeEqualityBound eq_y(DimensionIdx::Y, node_idx, DerivativeIdx::Position, pos.y);
	p4::NodeEqualityBound eq_z(DimensionIdx::Z, node_idx, DerivativeIdx::Position, pos.z);
	eq_constraints->push_back(eq_x);
	eq_constraints->push_back(eq_y);
	eq_constraints->push_back(eq_z);
}

void set_vel_eq_constraint(const geometry_msgs::Vector3 &vel, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints) {
	p4::NodeEqualityBound eq_x(DimensionIdx::X, node_idx, DerivativeIdx::Velocity, vel.x);
	p4::NodeEqualityBound eq_y(DimensionIdx::Y, node_idx, DerivativeIdx::Velocity, vel.y);
	p4::NodeEqualityBound eq_z(DimensionIdx::Z, node_idx, DerivativeIdx::Velocity, vel.z);
	eq_constraints->push_back(eq_x);
	eq_constraints->push_back(eq_y);
	eq_constraints->push_back(eq_z);
}

void set_acc_eq_constraint(const geometry_msgs::Vector3 &acc, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints) {
	p4::NodeEqualityBound eq_x(DimensionIdx::X, node_idx, DerivativeIdx::Acceleration, acc.x);
	p4::NodeEqualityBound eq_y(DimensionIdx::Y, node_idx, DerivativeIdx::Acceleration, acc.y);
	p4::NodeEqualityBound eq_z(DimensionIdx::Z, node_idx, DerivativeIdx::Acceleration, acc.z);
	eq_constraints->push_back(eq_x);
	eq_constraints->push_back(eq_y);
	eq_constraints->push_back(eq_z);
}

void set_max_vel(const double &max_vel, const uint &seg_idx,
	             std::vector<p4::SegmentInequalityBound> *ineq_constraints) {
	p4::SegmentInequalityBound ineq_x_max(seg_idx, DerivativeIdx::Velocity,
												       Eigen::Vector3d(1,0,0), max_vel);
	p4::SegmentInequalityBound ineq_y_max(seg_idx, DerivativeIdx::Velocity,
		                                               Eigen::Vector3d(0,1,0), max_vel);
	p4::SegmentInequalityBound ineq_z_max(seg_idx, DerivativeIdx::Velocity,
		                                               Eigen::Vector3d(0,0,1), max_vel);
	p4::SegmentInequalityBound ineq_x_min(seg_idx, DerivativeIdx::Velocity,
												       -Eigen::Vector3d(1,0,0), max_vel);
	p4::SegmentInequalityBound ineq_y_min(seg_idx, DerivativeIdx::Velocity,
		                                               -Eigen::Vector3d(0,1,0), max_vel);
	p4::SegmentInequalityBound ineq_z_min(seg_idx, DerivativeIdx::Velocity,
		                                               -Eigen::Vector3d(0,0,1), max_vel);
	ineq_constraints->push_back(ineq_x_max);
	ineq_constraints->push_back(ineq_y_max);
	ineq_constraints->push_back(ineq_z_max);
	ineq_constraints->push_back(ineq_x_min);
	ineq_constraints->push_back(ineq_y_min);
	ineq_constraints->push_back(ineq_z_min);
}

void set_max_acc(const double &max_acc, const uint &seg_idx,
	             std::vector<p4::SegmentInequalityBound> *ineq_constraints) {
	p4::SegmentInequalityBound ineq_x_max(seg_idx, DerivativeIdx::Acceleration,
												       Eigen::Vector3d(1,0,0), max_acc);
	p4::SegmentInequalityBound ineq_y_max(seg_idx, DerivativeIdx::Acceleration,
		                                               Eigen::Vector3d(0,1,0), max_acc);
	p4::SegmentInequalityBound ineq_z_max(seg_idx, DerivativeIdx::Acceleration,
		                                               Eigen::Vector3d(0,0,1), max_acc);
	p4::SegmentInequalityBound ineq_x_min(seg_idx, DerivativeIdx::Acceleration,
												       -Eigen::Vector3d(1,0,0), max_acc);
	p4::SegmentInequalityBound ineq_y_min(seg_idx, DerivativeIdx::Acceleration,
		                                               -Eigen::Vector3d(0,1,0), max_acc);
	p4::SegmentInequalityBound ineq_z_min(seg_idx, DerivativeIdx::Acceleration,
		                                               -Eigen::Vector3d(0,0,1), max_acc);
	ineq_constraints->push_back(ineq_x_max);
	ineq_constraints->push_back(ineq_y_max);
	ineq_constraints->push_back(ineq_z_max);
	ineq_constraints->push_back(ineq_x_min);
	ineq_constraints->push_back(ineq_y_min);
	ineq_constraints->push_back(ineq_z_min);
}

void set_max_jerk(const double &max_jerk, const uint &seg_idx,
	              std::vector<p4::SegmentInequalityBound> *ineq_constraints) {
	p4::SegmentInequalityBound ineq_x_max(seg_idx, DerivativeIdx::Jerk,
												       Eigen::Vector3d(1,0,0), max_jerk);
	p4::SegmentInequalityBound ineq_y_max(seg_idx, DerivativeIdx::Jerk,
		                                               Eigen::Vector3d(0,1,0), max_jerk);
	p4::SegmentInequalityBound ineq_z_max(seg_idx, DerivativeIdx::Jerk,
		                                               Eigen::Vector3d(0,0,1), max_jerk);
	p4::SegmentInequalityBound ineq_x_min(seg_idx, DerivativeIdx::Jerk,
												       -Eigen::Vector3d(1,0,0), max_jerk);
	p4::SegmentInequalityBound ineq_y_min(seg_idx, DerivativeIdx::Jerk,
		                                               -Eigen::Vector3d(0,1,0), max_jerk);
	p4::SegmentInequalityBound ineq_z_min(seg_idx, DerivativeIdx::Jerk,
		                                               -Eigen::Vector3d(0,0,1), max_jerk);
	ineq_constraints->push_back(ineq_x_max);
	ineq_constraints->push_back(ineq_y_max);
	ineq_constraints->push_back(ineq_z_max);
	ineq_constraints->push_back(ineq_x_min);
	ineq_constraints->push_back(ineq_y_min);
	ineq_constraints->push_back(ineq_z_min);
}

void setup_problem(const mg_msgs::minAccXYWpPVA::Request &req,
	               std::vector<double> *times,
				   std::vector<p4::NodeEqualityBound> *node_eq,
				   std::vector<p4::SegmentInequalityBound> *segment_ineq,
				   p4::PolynomialSolver::Options *solver_options) {
	const int n_w = req.PVA_array.size(); //Number of waypoints
	const int n_seg = n_w - 1;            //Number of polynomial segments

	// Set all equality constraints
	for (uint idx = 0; idx < n_w; idx++) {
		mg_msgs::PVA_request wp = req.PVA_array[idx];
		times->push_back(wp.time);
		if (wp.use_pos) {
			p4_helper::set_pos_eq_constraint(wp.Pos, idx, node_eq);
		}
		if (wp.use_vel) {
			p4_helper::set_vel_eq_constraint(wp.Vel, idx, node_eq);
		}
		if (wp.use_acc) {
			p4_helper::set_acc_eq_constraint(wp.Acc, idx, node_eq);
		}
	}

	// Set max vel, acc, and jerk for all segments
	for (uint idx = 0; idx < n_seg; idx++) {
		p4_helper::set_max_vel( req.max_vel,  idx, segment_ineq);
		p4_helper::set_max_acc( req.max_acc,  idx, segment_ineq);
		p4_helper::set_max_jerk(req.max_jerk, idx, segment_ineq);
	}

	// Configure solver options
	solver_options->num_dimensions   = 3;   // 3D
	solver_options->polynomial_order = 8;   // Fit an 8th-order polynomial
	solver_options->continuity_order = 4;   // Require continuity through the 4th derivative
	solver_options->derivative_order = 2;   // Minimize the 2nd derivative (acceleration)
	solver_options->polish = false;         // Polish the solution (osqp parameter)
}

mg_msgs::PolyPVA segment_pva_coeff_from_path(const p4::PolynomialPath &path,
	                                         const std::vector<double> &times,
	                                         const uint &seg_idx, const uint &dimension_idx) {
	mg_msgs::PolyPVA poly_pva;
	poly_pva.PosCoeff = 
		p4_helper::eigen_to_stdvector(path.coefficients[dimension_idx].col(seg_idx));
	poly_pva.VelCoeff = p4_helper::diff_coeff(poly_pva.PosCoeff);
	poly_pva.AccCoeff = p4_helper::diff_coeff(poly_pva.VelCoeff);
	poly_pva.order = poly_pva.PosCoeff.size() - 1;
	poly_pva.t0 = times[seg_idx];
	poly_pva.tf = times[seg_idx+1];
	return poly_pva;
}

Eigen::VectorXd time_to_segment_time(const std::vector<double> &times) {
	const uint n_w = times.size();  // Number of waypoints
	Eigen::VectorXd segment_times = Eigen::VectorXd::Zero(n_w-1);
	for (uint i = 0; i < n_w - 1; i++) {
		segment_times[i] = times[i+1] - times[i];
	}
	return segment_times;
}

std::vector<double> segment_time_to_time(const Eigen::VectorXd &segment_times) {
	std::vector<double> times;
	double cur_time = 0.0;
	times.push_back(cur_time);
	for (uint i = 0; i < segment_times.size(); i++) {
		cur_time = cur_time + segment_times[i];
		times.push_back(cur_time);
	}
	return times;
}

void solve_optimal_time_problem(const std::vector<double> &init_time_guess,
	                            const std::vector<p4::NodeEqualityBound> &node_eq,
								const std::vector<p4::SegmentInequalityBound> &segment_ineq,
								const p4::PolynomialSolver::Options &solver_options,
								const std::vector<p4::NodeInequalityBound> &node_ineq,
								std::vector<double> *times,
								p4::PolynomialPath *path) {
	// Variable declaration
	const uint n_w = init_time_guess.size();  // Number of waypoints
	Eigen::VectorXd segment_times = time_to_segment_time(init_time_guess);
	double cost;
	p4::PolynomialPath cur_trajectory;

	// Get initial solution for optimization problem
	p4::PolynomialSolver solver(solver_options);
	cur_trajectory = solver.Run(init_time_guess, node_eq, node_ineq, segment_ineq);
	// cost = path.optimal_cost;

	//Declare variables for gradient descent
	const double final_time = init_time_guess.back();
	const int m = segment_times.size();
	const double h = 0.00001;     //Small step for time gradient
	const double epsilon = 0.01;  //Condition for terminating gradient descent
	double step = std::numeric_limits<float>::infinity();
	double costNew;
	Eigen::VectorXd g = (-1.0/(m-1.0))*Eigen::MatrixXd::Ones(m,1);
	Eigen::VectorXd gradF = Eigen::MatrixXd::Zero(m,1);
	double init_cost = cost;
	Eigen::VectorXd best_segment_times;
	*path = cur_trajectory;

	//Gradient descent loop
	while(step > epsilon){

		//Calculate gradient
		Eigen::VectorXd g_i = g;
		Eigen::VectorXd segment_times_new = segment_times;
		for(int i = 0; i < m; i++){
			g_i = g;
			g_i[i] = 1;
			segment_times_new = segment_times + h*g_i;
			cur_trajectory = solver.Run(segment_time_to_time(segment_times_new), node_eq, node_ineq, segment_ineq);
			// costNew = cur_trajectory.optimal_cost;
			gradF(i) = (costNew - cost)/h;
		}

		//Normalize gradient vector (its bigger value will be as big as the bigger segment time)
		gradF = segment_times_new.cwiseAbs().minCoeff()*gradF/gradF.cwiseAbs().maxCoeff();

		//Perform gradient descent
		double alpha = 0.9;           //Step size
		double curCost = std::numeric_limits<float>::infinity();
		double prevCost = std::numeric_limits<float>::infinity();
		best_segment_times = segment_times;
		double bestCost = cost;
		for (int j = 0; j < 6; j++){  //Here we only iterate 6 times before alpha becomes too small
			segment_times_new = segment_times - alpha*gradF;

			//Renormalize segment times to preserve final time
			segment_times_new = final_time*segment_times_new/segment_times_new.sum();

			cur_trajectory = solver.Run(segment_time_to_time(segment_times_new), node_eq, node_ineq, segment_ineq);
			// curCost = cur_trajectory.optimal_cost;

			if(curCost < bestCost){
				bestCost = curCost;
				best_segment_times = segment_times_new;
			}

			alpha = alpha*0.5;
		    prevCost = curCost;
		}

		//Check if there was any improvement. Otherwise, stop iterations
		if(bestCost < cost){
			segment_times = best_segment_times;
			step = (cost - bestCost)/cost;
			cost = bestCost;
			*path = cur_trajectory;
		}
		else{
			break;
		}
	}

	*times = segment_time_to_time(segment_times);
}

}  // namespace p4_helper