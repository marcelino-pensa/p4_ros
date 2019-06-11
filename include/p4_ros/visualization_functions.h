#ifndef VISUALIZATION_FUNCTIONS_H_
#define VISUALIZATION_FUNCTIONS_H_

#include "ros/ros.h"
#include <Eigen/Dense>

// Ros message types
#include <visualization_msgs/Marker.h>
#include "p4_ros/PVA.h"

namespace visualization {

class TimeOptimizerPublisher {
 public:
 	TimeOptimizerPublisher();
 	TimeOptimizerPublisher(ros::NodeHandle *nh);
 	void VisualizePath(const Eigen::MatrixXd &polyCoeff,
 		               const Eigen::VectorXd &time);
 	void VisualizeWaypoints(const Eigen::MatrixXd &path);
 	void PubPVA_Markers(const geometry_msgs::Point &pos,
 		                const geometry_msgs::Vector3 &vel,
                        const geometry_msgs::Vector3 &acc);
 	void PubRealTimeTraj(const std::vector<p4_ros::PVA> &pva_vec,
 		                 const double &sampling_freq,
 		                 const double &final_time);
 	Eigen::Vector3d getPosPoly(const Eigen::MatrixXd &polyCoeff, const int &k,
 					           const double &t, const uint &n_coeff);

 private:
 	double vis_traj_width_;
 	ros::NodeHandle nh_;

 	// Visualization markers
	visualization_msgs::Marker vis_pos_, vis_vel_, vis_acc_;

	// ROS publishers
	ros::Publisher  wp_traj_vis_pub_, wp_path_vis_pub_;
    ros::Publisher  vis_pos_pub_, vis_vel_pub_, vis_acc_pub_;
};


}  // namespace visualization

#endif  // VISUALIZATION_FUNCTIONS_H_