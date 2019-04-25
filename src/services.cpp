
#include "p4_ros/services.h"

#include "gnuplot-iostream.h"

namespace p4_ros {

bool minAccXYService(mg_msgs::minAccXYWpPVA::Request  &req,
                     mg_msgs::minAccXYWpPVA::Response &res) {
	// Problem variables
	std::vector<double> times;
	std::vector<p4::NodeEqualityBound> node_eq;
	std::vector<p4::SegmentInequalityBound> segment_ineq;
	p4::PolynomialSolver::Options solver_options;
	std::vector<p4::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// Setup optimization problem
	p4_helper::setup_problem(req, &times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	p4::PolynomialSolver solver(solver_options);
	const p4::PolynomialPath path =
	    solver.Run(times, node_eq, node_ineq, segment_ineq);

	// Return structure
	const uint n_seg = times.size() - 1;
	for (uint idx = 0; idx < n_seg; idx++) {
		mg_msgs::PolyPVA X, Y;
		X = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::X);
		Y = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::Y);
		res.X.push_back(X);
		res.Y.push_back(Y);
		// std::cout << "poly " << idx << std::endl;
		// std::cout << path.coefficients[p4_helper::DimensionIdx::X].col(idx).transpose() << std::endl;
		// std::cout << path.coefficients[p4_helper::DimensionIdx::Y].col(idx).transpose() << std::endl;
	}
	// res.cost = path.optimal_cost;

	ROS_INFO("[p4_services] Returning request for %zu waypoints.", times.size());

	// {
	// p4::PolynomialSampler::Options sampler_options;
	// sampler_options.frequency        = 100;
	// sampler_options.derivative_order = 0;

	// p4::PolynomialSampler sampler(sampler_options);
	// Eigen::MatrixXd samples = sampler.Run(times, path);
 	
 //    std::vector<double> t_hist, x_hist, y_hist, z_hist;
 //    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
 //      t_hist.push_back(samples(0,time_idx));
 //      x_hist.push_back(samples(1,time_idx));
 //      y_hist.push_back(samples(2,time_idx));
 //      z_hist.push_back(samples(3,time_idx));
 //    }

 //    Gnuplot gp;
 //    gp << "plot '-' using 1:2 with lines title 'X-Position'";
 //    gp << ", '-' using 1:2 with lines title 'Y-Position'";
 //    gp << ", '-' using 1:2 with lines title 'Z-Position'";
 //    gp << std::endl;
 //    gp.send1d(boost::make_tuple(t_hist, x_hist));
 //    gp.send1d(boost::make_tuple(t_hist, y_hist));
 //    gp.send1d(boost::make_tuple(t_hist, z_hist));
 //    gp << "set grid" << std::endl;
 //    gp << "replot" << std::endl;
 //  }

	// {
	// p4::PolynomialSampler::Options sampler_options;
	// sampler_options.frequency        = 100;
	// sampler_options.derivative_order = 1;

	// p4::PolynomialSampler sampler(sampler_options);
	// Eigen::MatrixXd samples = sampler.Run(times, path);
 	
 //    std::vector<double> t_hist, x_hist, y_hist, z_hist;
 //    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
 //      t_hist.push_back(samples(0,time_idx));
 //      x_hist.push_back(samples(1,time_idx));
 //      y_hist.push_back(samples(2,time_idx));
 //      z_hist.push_back(samples(3,time_idx));
 //    }

 //    Gnuplot gp;
 //    gp << "plot '-' using 1:2 with lines title 'X-Velocity'";
 //    gp << ", '-' using 1:2 with lines title 'Y-Velocity'";
 //    gp << ", '-' using 1:2 with lines title 'Z-Velocity'";
 //    gp << std::endl;
 //    gp.send1d(boost::make_tuple(t_hist, x_hist));
 //    gp.send1d(boost::make_tuple(t_hist, y_hist));
 //    gp.send1d(boost::make_tuple(t_hist, z_hist));
 //    gp << "set grid" << std::endl;
 //    gp << "replot" << std::endl;
 //  }

	// {
	// p4::PolynomialSampler::Options sampler_options;
	// sampler_options.frequency        = 100;
	// sampler_options.derivative_order = 2;

	// p4::PolynomialSampler sampler(sampler_options);
	// Eigen::MatrixXd samples = sampler.Run(times, path);
 	
 //    std::vector<double> t_hist, x_hist, y_hist, z_hist;
 //    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
 //      t_hist.push_back(samples(0,time_idx));
 //      x_hist.push_back(samples(1,time_idx));
 //      y_hist.push_back(samples(2,time_idx));
 //      z_hist.push_back(samples(3,time_idx));
 //    }

 //    Gnuplot gp;
 //    gp << "plot '-' using 1:2 with lines title 'X-Acceleration'";
 //    gp << ", '-' using 1:2 with lines title 'Y-Acceleration'";
 //    gp << ", '-' using 1:2 with lines title 'Z-Acceleration'";
 //    gp << std::endl;
 //    gp.send1d(boost::make_tuple(t_hist, x_hist));
 //    gp.send1d(boost::make_tuple(t_hist, y_hist));
 //    gp.send1d(boost::make_tuple(t_hist, z_hist));
 //    gp << "set grid" << std::endl;
 //    gp << "replot" << std::endl;
 //  }

  // { // Plot 3D position
  //   p4::PolynomialSampler::Options sampler_options;
  //   sampler_options.frequency = 50;
  //   sampler_options.derivative_order = 0;

  //   p4::PolynomialSampler sampler(sampler_options);
  //   Eigen::MatrixXd samples = sampler.Run(times, path);

  //   std::vector<double> t_hist, x_hist, y_hist, z_hist;
  //   for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
  //     t_hist.push_back(samples(0,time_idx));
  //     x_hist.push_back(samples(1,time_idx));
  //     y_hist.push_back(samples(2,time_idx));
  //     z_hist.push_back(samples(3,time_idx));
  //   }

  //   Gnuplot gp;
  //   gp << "splot '-' using 1:2:3 with lines title 'Trajectory'" << std::endl;
  //   gp.send1d(boost::make_tuple(x_hist, y_hist, z_hist));
  //   gp << "set grid" << std::endl;
  //   gp << "replot" << std::endl;
  // }

	return true;
}

}  // namespace p4_ros