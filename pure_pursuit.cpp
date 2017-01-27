//pure_pursuit.cpp
#include <pure_pursuit/pure_pursuit.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(pure_pursuit, PurePursuit, pure_pursuit::PurePursuit, nav_core::BaseLocalPlanner)

namespace pure_pursuit{
    PurePursuit::PurePursuit(): tf_(NULL), costmap_ros_(NULL) {}
	
	//1 todos
	void PurePursuit::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
		tf_ = tf;
		costmap_ros_ = costmap_ros;
		current_waypoint_ = 0;
		goal_reached_time_ = ros::Time::now();
		ros::NodeHandle node_private("~/" + name);
		collision_planner_.initialize(name, tf_, costmap_ros_);
		//TODO:paramlar gelicek
		node_private.param("tolerance_timeout", tolerance_timeout_, 2.5);
		node_private.param("epsilon", epsilon_, 1e-6);
		node_private.param("frequency", frequency_, 20.0);
        node_private.param("initial_waypoint", initialWayPoint_, -1);
        node_private.param("look_ahead_ratio",lookAheadRatio_, 1.0);
        node_private.param
        
        ros::NodeHandle node;
        odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PoseFollower::odomCallback, this, _1));
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
		current_waypoint_ = 0;
		goal_reached_time_ = ros::Time::now();
		if(!transformGlobalPlan(*tf_, global_plan, *costmap_ros_, costmap_ros_->getGlobalFrameID(), global_plan_)){
			ROS_ERROR("Could not transform the global plan to the frame of the controller");
			return false;
		}
		return true;
	}
	
    
    int PurePursuit::getNextWayPoint(int wayPoint){
        
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
			
	//1 todo		
	bool PurePursuit::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
		//get the current pose of the robot in the fixed frame
		tf::Stamped<tf::Pose> robot_pose;
		if(!costmap_ros_->getRobotPose(robot_pose)){
		ROS_ERROR("Can't get robot pose");
		geometry_msgs::Twist empty_twist;
		cmd_vel = empty_twist;
		return false;
		}
		//waypoint
		tf::Stamped<tf::Pose> target_pose;
		tf::Stamped<tf::Pose> cur_pose_of_waypoints;
		tf::Stamped<tf::Pose> cur_pose_of_waypoints_next;
		tf::poseStampedMsgToTF(global_plan_[current_waypoint_], cur_pose_of_waypoints);
    
		//lookahead angle distance
		tf::Vector3 lookAheadDistance_ = getLookAheadDistance(cur_pose_of_waypoints, robot_pose);
		tf::Vector3 lookAheadAngle_ = getLookAheadAngle(cur_pose_of_waypoints, robot_pose);
    
		//arc distance
		double arcDistance=getArcDistance(lookAheadDistance_, lookAheadAngle_);
	
		//TODO:arcDistance to vel command
	}

	//ok sıkıntı olabilir, tf::poseda pose.position
	double::PurePursuit::getLookAheadDistance(const tf::Pose& pose1, const tf::Pose& pose2){
		tf::Vector3 v1(pose2.pose.position.x,pose2.pose.position.y,pose2.pose.position.z);
		tf::Vector3 v2(pose1.pose.position.x,pose1.pose.position.y,pose1.pose.position.z);
		return tf::tfDistance(v1, v2);
	}
	
	//ok sıkıntı olabilir, tf::poseda pose.position 
	double PurePursuit::getLookAheadAngle(const tf::Pose& pose1, const tf::Pose& pose2){
		tf::Vector3 v1(pose2.pose.position.x,pose2.pose.position.y,pose2.pose.position.z);
		tf::Vector3 v2(pose1.pose.position.x,pose1.pose.position.y,pose1.pose.position.z);
		return tf::tfAngle(v1, v2);
	}
	
	//ok
	double PurePursuit::getArcDistance(const tf::Vector3& lookAheadDistance,const tf::Vector3& lookAheadAngle){
		if (std::abs(std::sin(lookAheadAngle)) >= epsilon_)
			return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle;
		else
			return lookAheadDistance;
	}
	
	//ok -- cozuldu 
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
};
