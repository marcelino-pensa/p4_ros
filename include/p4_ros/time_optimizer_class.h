#ifndef TIME_OPTIMIZER_CLASS_H_
#define TIME_OPTIMIZER_CLASS_H_

#include "ros/ros.h"

// Locally defined
#include "p4_ros/PVA.h"
#include "p4_ros/p4_helper.h"

// TimeOptimizer libraries
#include "traj_poly_mono.h"
#include "time_optimizer.h"

namespace p4_ros {

class TimeOptimizerClass {
 public:
 	// Planning parameters
	double sampling_freq_;
	double max_vel_, max_acc_, max_jerk_, d_s_, rho_;
	uint poly_num_coeff_, num_segments_;

    // Trajectory to optimize on
    Eigen::MatrixXd polyCoeff_;
    Eigen::VectorXd polyTime_;

    // Time allocation structure (defined in timeAllocator.h)
    Allocator * time_allocator_ = new Allocator();

    // Trajectory's final time
    double final_time_;

	TimeOptimizerClass(const double &max_vel, const double &max_acc,
		               const double &max_jerk, const double &d_s,
		               const double &rho, const uint &poly_order,
		               const double &sampling_freq, const Eigen::MatrixXd &polyCoeff,
		               const Eigen::VectorXd &polyTime, const bool &visualize_output,
	                   std::vector<p4_ros::PVA> *pva_vec, float *final_time);

	void SolveMinTimeOpt();
	void GetTrajectoryPVA(std::vector<p4_ros::PVA> *pva_vec, float *final_time);
    Eigen::Vector3d getPosPoly(const Eigen::MatrixXd &polyCoeff, 
    	                       const int &k, const double &t);
    Eigen::Vector3d getVelPoly(const Eigen::MatrixXd &polyCoeff, 
    	                       const int &k, const double &t);
    Eigen::Vector3d getAccPoly(const Eigen::MatrixXd &polyCoeff,
    	                       const int &k, const double &t);
	void GetPVAatTime(const double &time_in, geometry_msgs::Point *pos,
				      geometry_msgs::Vector3 *vel, geometry_msgs::Vector3 *acc);
};

}  // namespace p4_ros

#endif  // TIME_OPTIMIZER_CLASS_H_