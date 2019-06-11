#include "p4_ros/visualization_functions.h"


namespace visualization {

TimeOptimizerPublisher::TimeOptimizerPublisher() { }

TimeOptimizerPublisher::TimeOptimizerPublisher(ros::NodeHandle *nh) {
    vis_traj_width_ = 0.15;

    // Declare trajectory publishers
    wp_traj_vis_pub_ = nh->advertise<visualization_msgs::Marker>("spatial_trajectory", 1);
    wp_path_vis_pub_ = nh->advertise<visualization_msgs::Marker>("waypoint_path"     , 1);
    vis_pos_pub_     = nh->advertise<visualization_msgs::Marker>("desired_position", 1);    
    vis_vel_pub_     = nh->advertise<visualization_msgs::Marker>("desired_velocity", 1);    
    vis_acc_pub_     = nh->advertise<visualization_msgs::Marker>("desired_acceleration", 1);
    std::cout << "position publisher: " << vis_pos_pub_.getTopic() << std::endl;

    // Set the structures for the path publishers
    vis_pos_.id = vis_vel_.id = vis_acc_.id = 0;
    vis_pos_.header.frame_id = vis_vel_.header.frame_id = vis_acc_.header.frame_id = "/map";
    
    vis_pos_.ns = "pos";
    vis_pos_.type   = visualization_msgs::Marker::SPHERE;
    vis_pos_.action = visualization_msgs::Marker::ADD;
    vis_pos_.color.a = 1.0; vis_pos_.color.r = 0.0; vis_pos_.color.g = 0.0; vis_pos_.color.b = 0.0;
    vis_pos_.scale.x = 0.2; vis_pos_.scale.y = 0.2; vis_pos_.scale.z = 0.2;

    vis_vel_.ns = "vel";
    vis_vel_.type = visualization_msgs::Marker::ARROW;
    vis_vel_.action = visualization_msgs::Marker::ADD;
    vis_vel_.color.a = 1.0; vis_vel_.color.r = 0.0; vis_vel_.color.g = 1.0; vis_vel_.color.b = 0.0;
    vis_vel_.scale.x = 0.2; vis_vel_.scale.y = 0.4; vis_vel_.scale.z = 0.4;

    vis_acc_.ns = "acc";
    vis_acc_.type = visualization_msgs::Marker::ARROW;
    vis_acc_.action = visualization_msgs::Marker::ADD;
    vis_acc_.color.a = 1.0; vis_acc_.color.r = 1.0; vis_acc_.color.g = 1.0; vis_acc_.color.b = 0.0;
    vis_acc_.scale.x = 0.2; vis_acc_.scale.y = 0.4; vis_acc_.scale.z = 0.4;
}

void TimeOptimizerPublisher::VisualizePath(const Eigen::MatrixXd &polyCoeff, const Eigen::VectorXd &time) {
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "time_optimal/trajectory_waypoints";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = vis_traj_width_;
    traj_vis.scale.y = vis_traj_width_;
    traj_vis.scale.z = vis_traj_width_;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;

    traj_vis.color.a = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Eigen::Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    traj_vis.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    const uint poly_num_coeff = polyCoeff.cols()/3;
    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = this->getPosPoly(polyCoeff, i, t, poly_num_coeff);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    wp_traj_vis_pub_.publish(traj_vis);
}

void TimeOptimizerPublisher::VisualizeWaypoints(const Eigen::MatrixXd &path) {
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    wp_path_vis_pub_.publish(points);
    wp_path_vis_pub_.publish(line_list);
}

// Publish trajectory (real-time visualization)
void TimeOptimizerPublisher::PubPVA_Markers(const geometry_msgs::Point &pos, const geometry_msgs::Vector3 &vel,
                                            const geometry_msgs::Vector3 &acc) {   

    vis_pos_.header.stamp = ros::Time::now();
    vis_vel_.header.stamp = ros::Time::now();
    vis_acc_.header.stamp = ros::Time::now();
    vis_pos_.points.clear();
    vis_vel_.points.clear();
    vis_acc_.points.clear();

    vis_pos_.points.push_back(pos);
    vis_vel_.points.push_back(pos);
    vis_acc_.points.push_back(pos);

    geometry_msgs::Point pt;
    pt.x = pos.x + vel.x;
    pt.y = pos.y + vel.y;
    pt.z = pos.z + vel.z;
    vis_vel_.points.push_back(pt);

    pt.x = pos.x + acc.x;
    pt.y = pos.y + acc.y;
    pt.z = pos.z + acc.z;
    vis_acc_.points.push_back(pt);
    
    vis_pos_pub_.publish(vis_pos_);
    vis_vel_pub_.publish(vis_vel_);
    vis_acc_pub_.publish(vis_acc_);
}

void TimeOptimizerPublisher::PubRealTimeTraj(const std::vector<p4_ros::PVA> &pva_vec,
                                             const double &sampling_freq,
                                             const double &final_time) {
    ROS_INFO("[p4_services] Publishing trajectory...");
    ros::Rate rate(sampling_freq);
    for (uint i = 0; i < pva_vec.size(); i++) {
        p4_ros::PVA cur_pva = pva_vec[i];
        printf("\rPublishing trajectory for time %4.2f/%4.2f sec", cur_pva.time, final_time);
        this->PubPVA_Markers(cur_pva.pos, cur_pva.vel, cur_pva.acc);
        rate.sleep();
    }
    printf("\n\r");
}

Eigen::Vector3d TimeOptimizerPublisher::getPosPoly(const Eigen::MatrixXd &polyCoeff, 
                           const int &k, const double &t, const uint &n_coeff) {
    Eigen::Vector3d ret;


    for ( int dim = 0; dim < 3; dim++ ) {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * n_coeff, n_coeff );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero( n_coeff );
        
        for(int j = 0; j < n_coeff; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

}  // namespace visualization