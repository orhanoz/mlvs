//pure_pursuit7.cpp
#include <pure_pursuit/pure_pursuit.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

PLUGINLIB_DECLARE_CLASS(pure_pursuit, PurePursuit, pure_pursuit::PurePursuit, nav_core::BaseLocalPlanner)

namespace pure_pursuit{
  PurePursuit::PurePursuit(): tf_(NULL), costmap_ros_(NULL) {}
	
  //ok
	void PurePursuit::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
		tf_ = tf;
		costmap_ros_ = costmap_ros;
		goal_reached_time_ = ros::Time::now();
		ros::NodeHandle node_private("~/" + name);
		collision_planner_.initialize(name, tf_, costmap_ros_);
        
		node_private.param("k_trans", K_trans_, 3.0);
    node_private.param("k_rot", K_rot_, 3.0);

    node_private.param("tolerance_trans", tolerance_trans_, 0.1);
    node_private.param("tolerance_rot", tolerance_rot_, 0.03);
    node_private.param("tolerance_timeout", tolerance_timeout_, 1.5);
    node_private.param("epsilon", epsilon_, 1e-6);
    node_private.param("lookAheadRatio",lookAheadRatio_, 1.0);
    node_private.param("holonomic", holonomic_, false);

    node_private.param("samples", samples_, 100);

    node_private.param("max_vel_lin", max_vel_lin_, 0.3);
    node_private.param("max_vel_th", max_vel_th_, 1.0);

    node_private.param("min_vel_lin", min_vel_lin_, 0.3);
    node_private.param("min_vel_th", min_vel_th_, -0.3);
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

    node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
    node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

    node_private.param("lookahead_count", lookahead_count_, 1);
    node_private.param("lookahead_weight", lookahead_weight_, 0.2);
        
    node_private.param("velocity_", velocity_, 0.2);
    node_private.param("max_heading_diff_before_moving", max_heading_diff_before_moving_, 0.17);
    node_private.param("turn_in_place_first", turn_in_place_first_, false);
        
    ros::NodeHandle node;
    odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PurePursuit::odomCallback, this, _1));
    vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);  
    ROS_DEBUG("Initialized");
	}
    
    //ok
	void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
		boost::mutex::scoped_lock lock(odom_lock_);
		base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
		base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
		base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
		ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
		base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
	}
    
    //ok
	bool PurePursuit::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
		goal_reached_time_ = ros::Time::now();
		if(!transformGlobalPlan(*tf_, global_plan, *costmap_ros_, costmap_ros_->getGlobalFrameID(), global_plan_)){
			ROS_ERROR("Could not transform the global plan to the frame of the controller");
			return false;
		}
		return true;
	}
    
    //ok
	bool PurePursuit::isGoalReached(){
		if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now() && stopped()){
			return true;
		}
		return false;
	}
	
	//ok
	bool PurePursuit::stopped(){
		//copy over the odometry information
		nav_msgs::Odometry base_odom;
		{
		boost::mutex::scoped_lock lock(odom_lock_);
		base_odom = base_odom_;
		}
		return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
		&& fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
		&& fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
	}
    
  //ok
  bool PurePursuit::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
    const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
		std::vector<geometry_msgs::PoseStamped>& transformed_plan){
			
		const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
		transformed_plan.clear();
		try{
			if (!global_plan.size() > 0){
				ROS_ERROR("Recieved plan with zero length");
				return false;
			}
  		tf::StampedTransform transform;
  		tf.lookupTransform(global_frame, ros::Time(), 
  			plan_pose.header.frame_id, plan_pose.header.stamp, 
  			plan_pose.header.frame_id, transform);
  				
  		tf::Stamped<tf::Pose> tf_pose;
  		geometry_msgs::PoseStamped newer_pose;
  		//now we'll transform until points are outside of our distance threshold
  		for(unsigned int i = 0; i < global_plan.size(); ++i){
  			const geometry_msgs::PoseStamped& pose = global_plan[i];
  			poseStampedMsgToTF(pose, tf_pose);
  			tf_pose.setData(transform * tf_pose);
  			tf_pose.stamp_ = transform.stamp_;
  			tf_pose.frame_id_ = global_frame;
  			poseStampedTFToMsg(tf_pose, newer_pose);
  			transformed_plan.push_back(newer_pose);
        //ROS_INFO("glob: %f %f %f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        //ROS_INFO("newer:%f %f %f",newer_pose.pose.position.x,newer_pose.pose.position.y,newer_pose.pose.position.z);
  		}
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			if (global_plan.size() > 0)
				ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
			return false;
		}
		return true;
	}

  bool PurePursuit::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    int wayPoint=getNextWP();
    ROS_INFO("WAYPOINT: %d",wayPoint);
    if(wayPoint>=0 && wayPoint<global_plan_.size()){
      //getCurrentPose
      tf::Stamped<tf::Pose> robot_pose;
      if(!costmap_ros_->getRobotPose(robot_pose)){
        ROS_ERROR("Can't get robot pose");
        geometry_msgs::Twist empty_twist;
        cmd_vel = empty_twist;
        return false;
      }
      //ROS_INFO("Current Pose: %f %f %f ",robot_pose.getOrigin().x(),robot_pose.getOrigin().y(),tf::getYaw(robot_pose.getRotation()));
      //getTargetPose
      tf::Stamped<tf::Pose> target_pose;
      tf::poseStampedMsgToTF(global_plan_[wayPoint], target_pose);
      //ROS_INFO("Current waypoint pose: %f %f %f",target_pose.getOrigin().x(),target_pose.getOrigin().y(),tf::getYaw(target_pose.getRotation()));
      
      //getVehicleCoord
      double dx = target_pose.getOrigin().x() - robot_pose.getOrigin().x();
      double dy = target_pose.getOrigin().y() - robot_pose.getOrigin().y();
      double x1 = std::cos(tf::getYaw(robot_pose.getRotation())) * dx + std::sin(tf::getYaw(robot_pose.getRotation())) * dy;
      double y1 = -1*std::sin(tf::getYaw(robot_pose.getRotation())) * dx + std::cos(tf::getYaw(robot_pose.getRotation())) * dy;
      double headi=atan2(y1,x1);
      //calcCurve
      double curve=2/((x1*x1)+(y1*y1))*(y1);//2/D^2*y1
      ROS_INFO("curve,headi:%f %f",curve,headi);
      double linearVelocity=0.1;
      double angularVelocity=0.0;

      if (headi>1.5 ){
        linearVelocity=0;
        angularVelocity=0.2;
      }
      else if(headi<-1.5 ){
        linearVelocity=0;
        angularVelocity=-0.2;
      }
      else{
        if(std::abs(headi)<epsilon_){
          ROS_INFO("epsilon");
          angularVelocity=0;
        }
        else
        angularVelocity = linearVelocity*curve;
      }
      geometry_msgs::Twist test_vel;
      test_vel.linear.x=linearVelocity;
      test_vel.linear.y=0;
      test_vel.angular.z=angularVelocity;
      bool legal_traj = collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, true);
      double scaling_factor = 1.0;
      double ds = scaling_factor / samples_;
      
      //let's make sure that the velocity command is legal... and if not, scale down
      if(!legal_traj){
        for(int i = 0; i < samples_; ++i){
          test_vel.linear.x = test_vel.linear.x * scaling_factor;
          test_vel.linear.y = test_vel.linear.y * scaling_factor;
          test_vel.angular.z = test_vel.angular.z * scaling_factor;
          //test_vel = limitTwist(test_vel);
          if(collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, false)){
            legal_traj = true;
            break;
          }
          scaling_factor -= ds;
        }
      }

      if(!legal_traj){
        ROS_ERROR("Not legal (%.2f, %.2f, %.2f)", test_vel.linear.x, test_vel.linear.y, test_vel.angular.z);
        geometry_msgs::Twist empty_twist;
        cmd_vel = empty_twist;
        return false;
      }

      //if it is legal... we'll pass it on
      cmd_vel = test_vel;

      //cmd_vel.linear.x=linearVelocity;
      //cmd_vel.angular.z=angularVelocity;
      
      bool in_goal_position=false;
      if(wayPoint==global_plan_.size()-1){
        in_goal_position=true;
        ROS_INFO("GOAL REACHED");
        geometry_msgs::Twist empty_twist;
        cmd_vel = empty_twist;
      }

      if(!in_goal_position)
        goal_reached_time_ = ros::Time::now();

      //check if we've reached our goal for long enough to succeed
      if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now()){
        geometry_msgs::Twist empty_twist;
        cmd_vel = empty_twist;
        ROS_INFO("HIZ 0'LAMA");
      }

      ROS_INFO("HOOOO: %f %f",cmd_vel.linear.x,cmd_vel.angular.z); 
      return true;     
    }
    return false;  
  }

   
  //error free, not tested
  int PurePursuit::getClosestWP(){
    if(global_plan_.size()>0){
      int closestWP=-1;
      double minDistance = -1.0;
      for(int i=0;i<global_plan_.size();++i){
        double distance=getLookAheadDistance(i); //arcdistance ??
        if((minDistance<0.0) || (distance<minDistance)){
          closestWP=i;
          minDistance=distance;
        }
      }
    return closestWP;
    }
    return -1;
  }
   
  //works but needed to be tested with linear velo other than 0
  int PurePursuit::getNextWP(){
    int closestWayPoint=getClosestWP();
    //ROS_INFO("closestWP: %d",closestWayPoint);
    if(global_plan_.size()>0 && closestWayPoint>=0){
      geometry_msgs::PoseStamped closest_waypoint_pose;
      tf::Vector3 v_1(global_plan_[closestWayPoint].pose.position.x,
        global_plan_[closestWayPoint].pose.position.y,
        global_plan_[closestWayPoint].pose.position.z);
      double lookAheadThreshold = getLookAheadThreshold();
      for (int i = closestWayPoint; i < global_plan_.size();++i){
        tf::Vector3 v_2(global_plan_[i].pose.position.x,
          global_plan_[i].pose.position.y,
          global_plan_[i].pose.position.z);
        //ROS_INFO("DIST&THRESH: %f %f",tf::tfDistance(v_1, v_2),lookAheadThreshold);
        if (tf::tfDistance(v_1, v_2) > lookAheadThreshold){
          //ROS_INFO("RETURNING: %d",i);
          return i;
        }
      }
      return closestWayPoint;
    }
    return -1;   
  }
   
  
  geometry_msgs::Twist PurePursuit::limitTwist(const geometry_msgs::Twist& twist){
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;
    if(!holonomic_)
      res.linear.y = 0.0;
    else    
      res.linear.y *= K_trans_;
    res.angular.z *= K_rot_;

    //if turn_in_place_first is true, see if we need to rotate in place to face our goal first
    if (holonomic_ || turn_in_place_first_ && fabs(twist.angular.z) > max_heading_diff_before_moving_)
    {
      res.linear.x = 0;
      res.linear.y = 0;
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      return res;
    }

    //make sure to bound things by our velocity limits
    double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
    double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
    if (lin_overshoot > 1.0) 
    {
      res.linear.x /= lin_overshoot;
      res.linear.y /= lin_overshoot;
    }

    //we only want to enforce a minimum velocity if we're not rotating in place
    if(lin_undershoot > 1.0)
    {
      res.linear.x *= lin_undershoot;
      res.linear.y *= lin_undershoot;
    }

    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);

    //we want to check for whether or not we're desired to rotate in place
    if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      res.linear.x = 0.0;
      res.linear.y = 0.0;
    }

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }

  double PurePursuit::headingDiff(double x, double y, double pt_x, double pt_y, double heading){
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }
    
  geometry_msgs::Twist PurePursuit::diff2D(const tf::Pose& pose1, const tf::Pose& pose2){
    geometry_msgs::Twist res;
    tf::Pose diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());

    if(fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_)
      return res;

    //in the case that we're not rotating to our goal position and we have a non-holonomic robot
    //we'll need to command a rotational velocity that will help us reach our desired heading
    
    //we want to compute a goal based on the heading difference between our pose and the target
    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
        pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

    //we'll also check if we can move more effectively backwards
    if (allow_backwards_) 
    {
      double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
          pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

      //check if its faster to just back up
      if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
        ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
        yaw_diff = neg_yaw_diff;
      }
    }

    //compute the desired quaterion
    tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
    tf::Quaternion rot = pose2.getRotation() * rot_diff;
    tf::Pose new_pose = pose1;
    new_pose.setRotation(rot);

    diff = pose2.inverse() * new_pose;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());
    return res;
  }


  //ok
  double PurePursuit::getLookAheadThreshold(){
    return lookAheadRatio_*base_odom_.twist.twist.linear.x;
  } 
    
  //ok & tested
  double PurePursuit::getLookAheadDistance(int waypoint){
    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)){
      ROS_ERROR("Can't get robot pose");
      return -1.0;
    }

    geometry_msgs::PoseStamped origin;
    poseStampedTFToMsg(robot_pose, origin);
        
    tf::Vector3 v1(origin.pose.position.x,
      origin.pose.position.y,
      origin.pose.position.z);
    tf::Vector3 v2(global_plan_[waypoint].pose.position.x,
      global_plan_[waypoint].pose.position.y,
      global_plan_[waypoint].pose.position.z);
        
    return tf::tfDistance(v1, v2);
  }
    
  //ok & tested
  double PurePursuit::getLookAheadAngle(int waypoint){
    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)){
      ROS_ERROR("Can't get robot pose");
      return -1.0;
    }
    geometry_msgs::PoseStamped origin;
    poseStampedTFToMsg(robot_pose, origin);
        
    tf::Vector3 v1(origin.pose.position.x,
      origin.pose.position.y,
      origin.pose.position.z);
    tf::Vector3 v2(global_plan_[waypoint].pose.position.x,
      global_plan_[waypoint].pose.position.y,
      global_plan_[waypoint].pose.position.z);
    
    return tf::tfAngle(v1, v2);
  }

  //ok & ??
  double PurePursuit::getArcDistance(int waypoint){
    double lookAheadDistance = getLookAheadDistance(waypoint);
    double lookAheadAngle = getLookAheadAngle(waypoint);

    if (std::abs(std::sin(lookAheadAngle)) >= epsilon_)
      return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle; //??
    else
      return lookAheadDistance;
  }

};






