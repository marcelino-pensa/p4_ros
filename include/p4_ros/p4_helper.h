
#ifndef P4_HELPER_H_
#define P4_HELPER_H_

#include <Eigen/Dense>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "polynomial_solver.h"
#include "mg_msgs/minAccXYWpPVA.h"

namespace p4_helper {

namespace DimensionIdx {
static constexpr int X = 0;
static constexpr int Y = 1;
static constexpr int Z = 2;
}

namespace DerivativeIdx {
static constexpr int Position = 0;
static constexpr int Velocity = 1;
static constexpr int Acceleration = 2;
static constexpr int Jerk = 3;
static constexpr int Snap = 4;
static constexpr int Crackle = 5;
static constexpr int Pop = 6;
}

std::vector<float> eigen_to_stdvector(const Eigen::VectorXd &eig_vec);

std::vector<float> diff_coeff(const std::vector<float> &coeff_in);

void set_pos_eq_constraint(const geometry_msgs::Point &pos, const uint &node_idx,
	                       std::vector<mediation_layer::NodeEqualityBound> *eq_constraints);

void set_vel_eq_constraint(const geometry_msgs::Vector3 &vel, const uint &node_idx,
	                       std::vector<mediation_layer::NodeEqualityBound> *eq_constraints);

void set_acc_eq_constraint(const geometry_msgs::Vector3 &acc, const uint &node_idx,
	                       std::vector<mediation_layer::NodeEqualityBound> *eq_constraints);

void set_max_vel(const double &max_vel, const uint &seg_idx,
	             std::vector<mediation_layer::SegmentInequalityBound> *ineq_constraints);

void set_max_acc(const double &max_acc, const uint &seg_idx,
	             std::vector<mediation_layer::SegmentInequalityBound> *ineq_constraints);

void set_max_jerk(const double &max_jerk, const uint &seg_idx,
	              std::vector<mediation_layer::SegmentInequalityBound> *ineq_constraints);

void setup_problem(const mg_msgs::minAccXYWpPVA::Request &req,
	               std::vector<double> *times,
				   std::vector<mediation_layer::NodeEqualityBound> *node_eq,
				   std::vector<mediation_layer::SegmentInequalityBound> *segment_ineq,
				   mediation_layer::PolynomialSolver::Options *solver_options);

Eigen::VectorXd time_to_segment_time(const std::vector<double> &times);

std::vector<double> segment_time_to_time(const Eigen::VectorXd &segment_times);

void solve_optimal_time_problem(const std::vector<double> &init_time_guess,
	                            const std::vector<mediation_layer::NodeEqualityBound> &node_eq,
								const std::vector<mediation_layer::SegmentInequalityBound> &segment_ineq,
								const mediation_layer::PolynomialSolver::Options &solver_options,
								const std::vector<mediation_layer::NodeInequalityBound> &node_ineq,
								std::vector<double> *times,
								mediation_layer::PolynomialPath *path);

mg_msgs::PolyPVA segment_pva_coeff_from_path(const mediation_layer::PolynomialPath &path,
	                                         const std::vector<double> &times,
	                                         const uint &seg_idx, const uint &dimension_idx);

}  // namespace p4_helper

#endif  // P4_HELPER_H_