
#include "p4_ros/p4_helper.h"

namespace p4_helper {

size_t factorial(size_t n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

std::vector<float> eigen_to_stdvector(const Eigen::VectorXd &eig_vec) {
	std::vector<float> vec;
	for (uint i = 0; i < eig_vec.size(); i++) {
		vec.push_back(eig_vec(i));
	}
	return vec;
}

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d &eig_vec) {
	geometry_msgs::Point pt;
	pt.x = eig_vec[0];
	pt.y = eig_vec[1];
	pt.z = eig_vec[2];
	return pt;
}

geometry_msgs::Point ros_point(const double &x, const double &y, const double &z) {
	geometry_msgs::Point pt;
	pt.x = x;
	pt.y = y;
	pt.z = z;
	return pt;
}

geometry_msgs::Vector3 ros_vector3(const double &x, const double &y, const double &z) {
	geometry_msgs::Vector3 vec;
	vec.x = x;
	vec.y = y;
	vec.z = z;
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

void get_perpendicular_vectors(const Eigen::Vector3d &v, Eigen::Vector3d *v1, Eigen::Vector3d *v2) {
	std::srand((unsigned int) time(0));
	Eigen::Vector3d rdn = Eigen::Vector3d::Random().normalized();
	Eigen::Vector3d v_bar = v.normalized();

	*v1 = (rdn - (rdn.dot(v_bar)*v_bar)).normalized();
	*v2 = v1->cross(v_bar);
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

void set_corridor_constraints(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1,
	                          const double &max_dist, const uint &seg_idx,
	                          std::vector<p4::SegmentInequalityBound> *ineq_constraints) {
	const Eigen::Vector3d pt0(p0.x, p0.y, p0.z);
	const Eigen::Vector3d pt1(p1.x, p1.y, p1.z);

	// If pt1 coincides with pt0, then we can't add corridor constraints
	if ((pt1 - pt0).norm() == 0) {
		return;
	}

	// Get vector along the segment from p0 to p1
	const Eigen::Vector3d v = (pt1 - pt0).normalized();

	// Get vectors perpendicular to v
	Eigen::Vector3d perp1, perp2;
	p4_helper::get_perpendicular_vectors(v, &perp1, &perp2);

	// Set normal vectors for corridor planes
	Eigen::Vector3d n1 = perp1, n2 = -perp1, n3 = perp2, n4 = -perp2;

	// Get points belonging to the corridor planes
	Eigen::Vector3d p_mid = 0.5*(pt0 + pt1);  // Point in the middle of the segment
	Eigen::Vector3d pp1 = p_mid - max_dist*n1;
	Eigen::Vector3d pp2 = p_mid - max_dist*n2;
	Eigen::Vector3d pp3 = p_mid - max_dist*n3;
	Eigen::Vector3d pp4 = p_mid - max_dist*n4;

	// Set containment within corridor
	p4::SegmentInequalityBound ineq_corridor1(seg_idx, DerivativeIdx::Position, -n1, -pp1.dot(n1));
	p4::SegmentInequalityBound ineq_corridor2(seg_idx, DerivativeIdx::Position, -n2, -pp2.dot(n2));
	p4::SegmentInequalityBound ineq_corridor3(seg_idx, DerivativeIdx::Position, -n3, -pp3.dot(n3));
	p4::SegmentInequalityBound ineq_corridor4(seg_idx, DerivativeIdx::Position, -n4, -pp4.dot(n4));
	ineq_constraints->push_back(ineq_corridor1);
	ineq_constraints->push_back(ineq_corridor2);
	ineq_constraints->push_back(ineq_corridor3);
	ineq_constraints->push_back(ineq_corridor4);
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

void setup_min_time_problem(const p4_ros::min_time::Request &req,
			               std::vector<double> *times,
						   std::vector<p4::NodeEqualityBound> *node_eq,
						   std::vector<p4::SegmentInequalityBound> *segment_ineq,
						   p4::PolynomialSolver::Options *solver_options) {
	const int n_w = req.pos_array.size(); //Number of waypoints
	const int n_seg = n_w - 1;            //Number of polynomial segments

	// Get times based on trapezoidal profile
	trapezoidal::time_from_waypoints(req.pos_array, req.max_vel, req.max_acc, times);

	// Set all equality constraints
	for (uint idx = 0; idx < n_w; idx++) {
		p4_helper::set_pos_eq_constraint(req.pos_array[idx], idx, node_eq);
		if ((idx == 0) || (idx == (n_w-1))){
			p4_helper::set_vel_eq_constraint(ros_vector3(0.0, 0.0, 0.0), idx, node_eq);
			p4_helper::set_acc_eq_constraint(ros_vector3(0.0, 0.0, 0.0), idx, node_eq);
		}

		if(idx != 0) {
			set_corridor_constraints(req.pos_array[idx-1], req.pos_array[idx],
	                          req.corridor_width, idx-1, segment_ineq);
		}
	}

	// Configure solver options
	solver_options->num_dimensions   = 3;         // 3D
	solver_options->polynomial_order = 7;         // Fit an 8th-order polynomial
	solver_options->continuity_order = 4;         // Require continuity through the 4th derivative
	solver_options->derivative_order = 1;         // Minimize the 2nd derivative (acceleration)
	solver_options->num_intermediate_points = 10; // Number of points in segment constraints
	solver_options->polish = false;               // Polish the solution (osqp parameter)
}

void setup_min_acc_problem(const p4_ros::minAccXYWpPVA::Request &req,
			               std::vector<double> *times,
						   std::vector<p4::NodeEqualityBound> *node_eq,
						   std::vector<p4::SegmentInequalityBound> *segment_ineq,
						   p4::PolynomialSolver::Options *solver_options) {
	const int n_w = req.PVA_array.size(); //Number of waypoints
	const int n_seg = n_w - 1;            //Number of polynomial segments

	// Set all equality constraints
	for (uint idx = 0; idx < n_w; idx++) {
		p4_ros::PVA_request wp = req.PVA_array[idx];
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

p4_ros::PolyPVA segment_pva_coeff_from_path(const p4::PolynomialPath &path,
	                                         const std::vector<double> &times,
	                                         const uint &seg_idx, const uint &dimension_idx) {
	p4_ros::PolyPVA poly_pva;
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
	ROS_WARN("Get first solution!");
	cur_trajectory = solver.Run(init_time_guess, node_eq, node_ineq, segment_ineq);
	cost = cur_trajectory.optimal_cost;

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
	// *path = cur_trajectory;

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
			costNew = cur_trajectory.optimal_cost;
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
		for (int j = 0; j < 8; j++){  //Here we only iterate 6 times before alpha becomes too small
			segment_times_new = segment_times - alpha*gradF;

			//Renormalize segment times to preserve final time
			segment_times_new = final_time*segment_times_new/segment_times_new.sum();

			cur_trajectory = solver.Run(segment_time_to_time(segment_times_new), node_eq, node_ineq, segment_ineq);
			curCost = cur_trajectory.optimal_cost;

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
			// *path = cur_trajectory;
		}
		else{
			break;
		}
	}

	*times = segment_time_to_time(segment_times);

	// Get a polished solution
	p4::PolynomialSolver::Options solver_options_polished = solver_options;
	solver_options_polished.polish = true;
	p4::PolynomialSolver solver_polished(solver_options_polished);
	*path = solver_polished.Run(*times, node_eq, node_ineq, segment_ineq);
}

void plot_results(const std::vector<double> &times, const p4::PolynomialPath &path) {
	{
		p4::PolynomialSampler::Options sampler_options;
		sampler_options.frequency        = 100;
		sampler_options.derivative_order = 0;

		p4::PolynomialSampler sampler(sampler_options);
		Eigen::MatrixXd samples = sampler.Run(times, path);
	 	
	    std::vector<double> t_hist, x_hist, y_hist, z_hist;
	    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
	      t_hist.push_back(samples(0,time_idx));
	      x_hist.push_back(samples(1,time_idx));
	      y_hist.push_back(samples(2,time_idx));
	      z_hist.push_back(samples(3,time_idx));
	    }

	    Gnuplot gp;
	    gp << "plot '-' using 1:2 with lines title 'X-Position'";
	    gp << ", '-' using 1:2 with lines title 'Y-Position'";
	    gp << ", '-' using 1:2 with lines title 'Z-Position'";
	    gp << std::endl;
	    gp.send1d(boost::make_tuple(t_hist, x_hist));
	    gp.send1d(boost::make_tuple(t_hist, y_hist));
	    gp.send1d(boost::make_tuple(t_hist, z_hist));
	    gp << "set grid" << std::endl;
	    gp << "replot" << std::endl;
	}

	{
		p4::PolynomialSampler::Options sampler_options;
		sampler_options.frequency        = 100;
		sampler_options.derivative_order = 1;

		p4::PolynomialSampler sampler(sampler_options);
		Eigen::MatrixXd samples = sampler.Run(times, path);
	 	
	    std::vector<double> t_hist, x_hist, y_hist, z_hist;
	    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
	      t_hist.push_back(samples(0,time_idx));
	      x_hist.push_back(samples(1,time_idx));
	      y_hist.push_back(samples(2,time_idx));
	      z_hist.push_back(samples(3,time_idx));
	    }

	    Gnuplot gp;
	    gp << "plot '-' using 1:2 with lines title 'X-Velocity'";
	    gp << ", '-' using 1:2 with lines title 'Y-Velocity'";
	    gp << ", '-' using 1:2 with lines title 'Z-Velocity'";
	    gp << std::endl;
	    gp.send1d(boost::make_tuple(t_hist, x_hist));
	    gp.send1d(boost::make_tuple(t_hist, y_hist));
	    gp.send1d(boost::make_tuple(t_hist, z_hist));
	    gp << "set grid" << std::endl;
	    gp << "replot" << std::endl;
    }

	{
		p4::PolynomialSampler::Options sampler_options;
		sampler_options.frequency        = 100;
		sampler_options.derivative_order = 2;

		p4::PolynomialSampler sampler(sampler_options);
		Eigen::MatrixXd samples = sampler.Run(times, path);
	 	
	    std::vector<double> t_hist, x_hist, y_hist, z_hist;
	    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
	      t_hist.push_back(samples(0,time_idx));
	      x_hist.push_back(samples(1,time_idx));
	      y_hist.push_back(samples(2,time_idx));
	      z_hist.push_back(samples(3,time_idx));
	    }

	    Gnuplot gp;
	    gp << "plot '-' using 1:2 with lines title 'X-Acceleration'";
	    gp << ", '-' using 1:2 with lines title 'Y-Acceleration'";
	    gp << ", '-' using 1:2 with lines title 'Z-Acceleration'";
	    gp << std::endl;
	    gp.send1d(boost::make_tuple(t_hist, x_hist));
	    gp.send1d(boost::make_tuple(t_hist, y_hist));
	    gp.send1d(boost::make_tuple(t_hist, z_hist));
	    gp << "set grid" << std::endl;
	    gp << "replot" << std::endl;
	}
}

void plot_results_3d(const std::vector<double> &times, const p4::PolynomialPath &path) {
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 50;
    sampler_options.derivative_order = 0;

    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    std::vector<double> t_hist, x_hist, y_hist, z_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
      z_hist.push_back(samples(3,time_idx));
    }

    Gnuplot gp;
    gp << "splot '-' using 1:2:3 with lines title 'Trajectory'" << std::endl;
    gp.send1d(boost::make_tuple(x_hist, y_hist, z_hist));
    gp << "set grid" << std::endl;
    gp << "replot" << std::endl;

    std::cout << "Press enter to exit." << std::endl;
	std::cin.get();
}

}  // namespace p4_helper