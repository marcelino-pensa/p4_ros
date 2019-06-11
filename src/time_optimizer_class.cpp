#include "p4_ros/time_optimizer_class.h"


namespace p4_ros {

TimeOptimizerClass::TimeOptimizerClass(
	               const double &max_vel, const double &max_acc,
	               const double &max_jerk, const double &d_s,
	               const double &rho, const uint &poly_order,
	               const double &sampling_freq, const Eigen::MatrixXd &polyCoeff,
	               const Eigen::VectorXd &polyTime, const bool &visualize_output,
                   std::vector<p4_ros::PVA> *pva_vec, float *final_time) {
	ros::NodeHandle nh("~");
	max_vel_ = max_vel;
	max_acc_ = max_acc;
	max_jerk_ = max_jerk;
	d_s_ = d_s;
	rho_ = rho;
	poly_num_coeff_ = poly_order + 1;
	vis_traj_width_ = 0.15;
	polyCoeff_ = polyCoeff;
	polyTime_ = polyTime;
	num_segments_ = polyTime.size() - 1;
	sampling_freq_ = sampling_freq;

    if(sampling_freq_ <= 0) {
        sampling_freq_ = 50;
    }

    // Sets publishers and visualization markers
    this->SetVisualizationMarkerStructures(&nh);

    // Solve minimum time optimization problem
    this->SolveMinTimeOpt();

    // Retrieve position, velocity and acceleration from optimal solution
    this->GetTrajectoryPVA(pva_vec, final_time);

    // Quick nap until the publishers can finally publish
    sleep(1);

    // visualize the spatial fixed trajectory
    visWayPointTraj(polyCoeff_, polyTime_);

    // Publish a "real-time" visualization of the trajectory
    if (visualize_output) {
        const double tf = *final_time;
        ROS_INFO("Final time: %f", tf);
        ros::Rate rate(sampling_freq_);
        for (uint i = 0; i < pva_vec->size(); i++) {
            printf("\rPublishing trajectory for time %4.2f/%4.2f", (*pva_vec)[i].time, tf);
            this->pubCmd((*pva_vec)[i].pos, (*pva_vec)[i].vel, (*pva_vec)[i].acc);
            rate.sleep();
        }
        printf("\n\r");
    }
}

void TimeOptimizerClass::SolveMinTimeOpt() {
	// Structure for the time optimizer
	TrajPolyMono polyTraj(polyCoeff_, polyTime_);

	// run the time optimizer
	MinimumTimeOptimizer time_optimizer;
	ros::Time time_3 = ros::Time::now();
    if(time_optimizer.MinimumTimeGeneration( polyTraj, max_vel_, max_acc_, max_jerk_, d_s_, rho_)) {   
        ros::Time time_4 = ros::Time::now();
        // _has_traj = true;    
        ROS_WARN("[TimeOptimizer DEMO] Temporal trajectory generated");
        cout<<"[TimeOptimizer DEMO] time spent in temporal trajectory is: "<<(time_4 - time_3).toSec()<<endl;
        
        // pull out the results in an allocator data structure
        time_allocator_ = time_optimizer.GetTimeAllocation();

        traj_time_final_ = traj_time_start_ = ros::Time::now();
        for(int i = 0; i < time_allocator_->time.rows(); i++)
        {   
            int K = time_allocator_->K(i);
            traj_time_final_ += ros::Duration(time_allocator_->time(i, K - 1));
        }

        cout<<"[TimeOptimizer DEMO] now start publishing commands"<<endl;
    }
    else
    {
        cout<<"[TimeOptimizer DEMO] temporal optimization fail"<<endl;
        cout<<"[TimeOptimizer DEMO] possible resons : " << "\n" <<
        "1 - please check the spatial trajectory,"     <<  "\n" <<
        "2 - numerical issue of the solver, try setting a larger d_s"<<endl;
    }
}

void TimeOptimizerClass::GetTrajectoryPVA(std::vector<p4_ros::PVA> *pva_vec, float *final_time) {
    const double tf = (traj_time_final_ - traj_time_start_).toSec();
    const double dt = 1.0/sampling_freq_;
    geometry_msgs::Point pos;
    geometry_msgs::Vector3 vel, acc;
    double t = 0.0;
    p4_ros::PVA pva;
    while (t < tf) {
        this->GetPVAatTime(t, &pva.pos, &pva.vel, &pva.acc);
        pva.time = t;
        pva_vec->push_back(pva);
        t = t + dt;
    }
    this->GetPVAatTime(tf, &pva.pos, &pva.vel, &pva.acc);
    pva.time = tf;
    pva.vel = p4_helper::ros_vector3(0.0, 0.0, 0.0);
    pva.acc = p4_helper::ros_vector3(0.0, 0.0, 0.0);
    pva_vec->push_back(pva);
    *final_time = tf;
}

void TimeOptimizerClass::SetVisualizationMarkerStructures(ros::NodeHandle *nh) {
    // Declare trajectory publishers
    wp_traj_vis_pub_ = nh->advertise<visualization_msgs::Marker>("spatial_trajectory", 1);
    wp_path_vis_pub_ = nh->advertise<visualization_msgs::Marker>("waypoint_path"     , 1);
    vis_pos_pub_     = nh->advertise<visualization_msgs::Marker>("desired_position", 1);    
    vis_vel_pub_     = nh->advertise<visualization_msgs::Marker>("desired_velocity", 1);    
    vis_acc_pub_     = nh->advertise<visualization_msgs::Marker>("desired_acceleration", 1);

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

void TimeOptimizerClass::visWayPointTraj(const Eigen::MatrixXd &polyCoeff, const Eigen::VectorXd &time) {
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
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    traj_vis.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
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

void TimeOptimizerClass::visWayPointPath(const Eigen::MatrixXd &path) {
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

Eigen::Vector3d TimeOptimizerClass::getPosPoly(const Eigen::MatrixXd &polyCoeff, 
	                       const int &k, const double &t) {
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ ) {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num_coeff_, poly_num_coeff_ );
        Eigen::VectorXd time  = VectorXd::Zero( poly_num_coeff_ );
        
        for(int j = 0; j < poly_num_coeff_; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Eigen::Vector3d TimeOptimizerClass::getVelPoly(const Eigen::MatrixXd &polyCoeff, 
	                       const int &k, const double &t) {
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num_coeff_, poly_num_coeff_ );
        Eigen::VectorXd time  = VectorXd::Zero( poly_num_coeff_ );
        
        for(int j = 0; j < poly_num_coeff_; j ++)
            if(j==0)
                time(j) = 0.0;
            else
                time(j) = j * pow(t, j-1);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Eigen::Vector3d TimeOptimizerClass::getAccPoly(const Eigen::MatrixXd &polyCoeff,
	                       const int &k, const double &t) {
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num_coeff_, poly_num_coeff_ );
        Eigen::VectorXd time  = VectorXd::Zero( poly_num_coeff_ );

        for(int j = 0; j < poly_num_coeff_; j ++)
            if( j==0 || j==1 )
                time(j) = 0.0;
            else
                time(j) = j * (j - 1) * pow(t, j-2);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

// Publish trajectory (real-time visualization)
void TimeOptimizerClass::pubCmd(const geometry_msgs::Point &pos, const geometry_msgs::Vector3 &vel,
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

void TimeOptimizerClass::GetPVAatTime(
				const double &time_in, geometry_msgs::Point *pos,
				geometry_msgs::Vector3 *vel, geometry_msgs::Vector3 *acc) {
	Eigen::MatrixXd time     = time_allocator_->time;
    Eigen::MatrixXd time_acc = time_allocator_->time_acc;
    double t = time_in;

    int idx;
    for(idx = 0; idx < num_segments_; idx++)
    {   
        int K = time_allocator_->K(idx);
        if( t  > time(idx, K - 1))
            t -= time(idx, K - 1);
        else
            break;
    }
    double t_tmp = t;     

    int grid_num = time_allocator_->K(idx);
    
    // now we need to find which grid the time instance belongs to
    int grid_idx;
    for(grid_idx = 0; grid_idx < time_allocator_->K(idx); grid_idx++){
        if (t > time(idx, grid_idx)) continue;
        else{ 
            if(grid_idx > 0) t -= time(idx, grid_idx - 1);
            else             t -= 0.0;
            break;
        }
    }
    
    double delta_t;
    if(grid_idx > 0){	
      delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
    } else {
      delta_t = time(idx, grid_idx) - 0.0;
    }
    
    double delta_s = t * time_allocator_->s_step / delta_t;
    double s = time_allocator_->s(idx, grid_idx) + delta_s;

    // get position data 
    Eigen::Vector3d position   = getPosPoly(polyCoeff_, idx, s);

    // get velocity data
    double s_k   = time_allocator_->s(idx, grid_idx);
    double s_k_1 = time_allocator_->s(idx, grid_idx + 1);
    double b_k   = time_allocator_->b(idx, grid_idx);
    double b_k_1 = time_allocator_->b(idx, grid_idx + 1);

    Vector3d velocity_s1 = getVelPoly(polyCoeff_, idx, s_k  ); 
    Vector3d velocity_s2 = getVelPoly(polyCoeff_, idx, s_k_1);

    Vector3d velocity1   = velocity_s1 * sqrt(b_k);
    Vector3d velocity2   = velocity_s2 * sqrt(b_k_1);
    Vector3d velocity   = velocity1 + (velocity2 - velocity1) * t / delta_t;

// ### NOTE: From what above we get the position and velocity easily.
// ###       positions are the same as the trajectory before re-timing; and velocity are obtained by interpolation between each grid.
// ###       In what follows, we will get the accleration. It's more complicated since each acceleration ais evaluated at the middle of a grid        
    // reset grid_idx and t for time acceleration axis
    t = t_tmp;
    for(grid_idx = 0; grid_idx < time_allocator_->K(idx); grid_idx++)
    {
        if (t > time_acc(idx, grid_idx)) continue;
        else{ 
            if(grid_idx > 0) t -= time_acc(idx, grid_idx - 1);
            else             t -= 0.0;
            break;
        }
    }
    
    if(grid_idx == grid_num)
        t -= time_acc(idx, grid_num - 1);

    // prepare to do accleration interpolation
    Vector3d velocity_s, acceleration_s, acceleration1, acceleration2;
    Vector3d acceleration;

    double a_k;
    // # special case 1: the very first grid of all segments of the trajectory, do interpolation in one grid
    if( grid_idx == 0 && idx == 0 ) 
    {   
        s_k   = time_allocator_->s(idx, 0);
        s_k_1 = time_allocator_->s(idx, 0 + 1);
        
        a_k   = time_allocator_->a(idx, 0);
        b_k   = time_allocator_->b(idx, 0);
        b_k_1 = time_allocator_->b(idx, 0 + 1);

        velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
        acceleration1 << 0.0, 0.0, 0.0;
        
        acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / time_acc(0, 0); 
    }
    // # special case 2: the very last grid of all segments of the trajectory, do interpolation in one grid
    else if( grid_idx == grid_num && idx == (num_segments_ - 1) )
    {   
        s_k   = time_allocator_->s(idx, grid_num - 1);
        s_k_1 = time_allocator_->s(idx, grid_num);
        
        a_k   = time_allocator_->a(idx, grid_num - 1);
        b_k   = time_allocator_->b(idx, grid_num - 1);
        b_k_1 = time_allocator_->b(idx, grid_num    );

        velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
        acceleration = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
    }
    // # regular case: do interpolation between two grids
    else 
    {   
        // sub-case 1: two grids are in the same segment
        if(grid_idx < grid_num && grid_idx > 0) // take average accleration in a same segment
        {   
            delta_t = (time_acc(idx, grid_idx) - time_acc(idx, grid_idx - 1));
            
            s_k   = time_allocator_->s(idx, grid_idx - 1);
            s_k_1 = time_allocator_->s(idx, grid_idx + 0);
            
            a_k   = time_allocator_->a(idx, grid_idx - 1);
            b_k   = time_allocator_->b(idx, grid_idx - 1);
            b_k_1 = time_allocator_->b(idx, grid_idx + 0);

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

            s_k   = time_allocator_->s(idx, grid_idx + 0);
            s_k_1 = time_allocator_->s(idx, grid_idx + 1);

            a_k   = time_allocator_->a(idx, grid_idx + 0);
            b_k   = time_allocator_->b(idx, grid_idx + 0);
            b_k_1 = time_allocator_->b(idx, grid_idx + 1);              

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;   
        }
        // sub-case 2: two grids are in consecutive segment, the current grid is in a segment's tail
        else if(grid_idx == grid_num)// take average accleration between two segments
        {   
            delta_t = (time(idx, grid_num - 1) - time_acc(idx, grid_num - 1) + time_acc(idx + 1, 0) );
            
            s_k   = time_allocator_->s(idx, grid_idx - 1);
            s_k_1 = time_allocator_->s(idx, grid_idx);
            
            a_k   = time_allocator_->a(idx, grid_idx - 1);
            b_k   = time_allocator_->b(idx, grid_idx - 1);
            b_k_1 = time_allocator_->b(idx, grid_idx);

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            s_k   = time_allocator_->s(idx + 1, 0);
            s_k_1 = time_allocator_->s(idx + 1, 1);

            a_k   = time_allocator_->a(idx + 1, 0);
            b_k   = time_allocator_->b(idx + 1, 0);
            b_k_1 = time_allocator_->b(idx + 1, 1);              

            velocity_s     = getVelPoly(polyCoeff_, idx + 1, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx + 1, (s_k + s_k_1 ) / 2.0);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration  = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;        
        }
        // sub-case 3: two grids are in consecutive segment, the current grid is in a segment's head
        else if(grid_idx == 0)// take average accleration between two segments
        {   
            int grid_num_k = time_allocator_->K(idx - 1);
            delta_t = (time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1) + time_acc(idx, 0) );
            
            s_k   = time_allocator_->s(idx - 1, grid_num_k - 1);
            s_k_1 = time_allocator_->s(idx - 1, grid_num_k    );
            
            a_k   = time_allocator_->a(idx - 1, grid_num_k - 1);
            b_k   = time_allocator_->b(idx - 1, grid_num_k - 1);
            b_k_1 = time_allocator_->b(idx - 1, grid_num_k    );

            velocity_s     = getVelPoly(polyCoeff_, idx - 1, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx - 1, (s_k + s_k_1 ) / 2.0);
            acceleration1  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

            s_k   = time_allocator_->s(idx, 0);
            s_k_1 = time_allocator_->s(idx, 0 + 1);
            
            a_k   = time_allocator_->a(idx, 0);
            b_k   = time_allocator_->b(idx, 0);
            b_k_1 = time_allocator_->b(idx, 0 + 1);

            velocity_s     = getVelPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(polyCoeff_, idx, (s_k + s_k_1 ) / 2.0);
            acceleration2  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * (t + time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1)) / delta_t;   
        } 
        else {
            // no else
        }
    }

    pos->x = position(0);
    pos->y = position(1);
    pos->z = position(2);
    vel->x = velocity(0);
    vel->y = velocity(1);
    vel->z = velocity(2);
    acc->x = acceleration(0);
    acc->y = acceleration(1);
    acc->z = acceleration(2);
}

}  // namespace p4_ros