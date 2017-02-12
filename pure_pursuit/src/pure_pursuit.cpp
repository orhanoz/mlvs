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
		current_waypoint_ = 0;
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
    if(wayPoint>=0){
      //getCurrentPose
      tf::Stamped<tf::Pose> robot_pose;
      if(!costmap_ros_->getRobotPose(robot_pose)){
        ROS_ERROR("Can't get robot pose");
        geometry_msgs::Twist empty_twist;
        cmd_vel = empty_twist;
        return false;
      }
      ROS_INFO("Current Pose: %f %f %f ",robot_pose.getOrigin().x(),robot_pose.getOrigin().y(),tf::getYaw(robot_pose.getRotation()));
      //getTargetPose
      tf::Stamped<tf::Pose> target_pose;
      tf::poseStampedMsgToTF(global_plan_[wayPoint], target_pose);
      ROS_INFO("Current waypoint pose: %f %f %f",target_pose.getOrigin().x(),target_pose.getOrigin().y(),tf::getYaw(target_pose.getRotation()));
      
      //getVehicleCoord
      double dx = target_pose.getOrigin().x() - robot_pose.getOrigin().x();
      double dy = target_pose.getOrigin().y() - robot_pose.getOrigin().y();
      double x1 = std::cos(tf::getYaw(robot_pose.getRotation())) * dx + std::sin(tf::getYaw(robot_pose.getRotation())) * dy;
      double y1 = -1*std::sin(tf::getYaw(robot_pose.getRotation())) * dx + std::cos(tf::getYaw(robot_pose.getRotation())) * dy;

      //calcCurve
      double curve=2/((x1*x1)+(y1*y1))*(-1*y1);
      ROS_INFO("x1,y1,curve: %f %f %f",x1,y1,curve);
      double linearVelocity=0.1;
      double angularVelocity=0.0;
      double lookAheadAngle=getLookAheadAngle(wayPoint);
      double lookAheadDistance=getLookAheadDistance(wayPoint);
            
      //double radius = 0.5*(lookAheadDistance/std::sin(lookAheadAngle));
      //angularVelocity= sign(curve)*std::max((linearVelocity*curve),min_vel_th_);
      if (std::abs(std::sin(lookAheadAngle)) >= epsilon_) {
          double radius = 0.5*(lookAheadDistance/std::sin(lookAheadAngle));
          double linearVelocity = 0.2;
          if (std::abs(radius) >= epsilon_)
            angularVelocity = linearVelocity/radius;
      }
      cmd_vel.linear.x=linearVelocity;
      cmd_vel.angular.z=angularVelocity;
      ROS_INFO("HOOOO: %f %f",cmd_vel.linear.x,cmd_vel.angular.z);
      
   }
       
  }

   
  //error free, not tested
  int PurePursuit::getClosestWP(){
    if(global_plan_.size()>0){
      int closestWP=-1;
      double minDistance = -1.0;
      for(int i=0;i<global_plan_.size();++i){
        double distance=getArcDistance(i);
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
    ROS_INFO("closestWP: %d",closestWayPoint);
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
        ROS_INFO("DIST&THRESH: %f %f",tf::tfDistance(v_1, v_2),lookAheadThreshold);
        if (tf::tfDistance(v_1, v_2) > lookAheadThreshold){
          ROS_INFO("RETURNING: %d",i);
          return i;
        }
      }
      return closestWayPoint;
    }
    return -1;   
  }
   
   
   
   
   
   
  //ok ama kusuratta fark cikiyor
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

  //ok & tested
  double PurePursuit::getArcDistance(int waypoint){
    double lookAheadDistance = getLookAheadDistance(waypoint);
    double lookAheadAngle = getLookAheadAngle(waypoint);

    if (std::abs(std::sin(lookAheadAngle)) >= epsilon_)
      return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle;
    else
      return lookAheadDistance;
  }

};






