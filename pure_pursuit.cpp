//pure_pursuit.cpp
#include <pure_pursuit/pure_pursuit.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(pure_pursuit, PurePursuit, pure_pursuit::PurePursuit, nav_core::BaseLocalPlanner)

namespace pure_pursuit{
    PurePursuit::PurePursuit(): tf_(NULL), costmap_ros_(NULL) {}
    //1 todos
    void PurePursuit::initialize(std::string name, tf::TransformListener* tf , costmap_2d::Costmap2DROS* costmap_ros){
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
        node_private.param("samples", samples_, 100);
        node_private.param("tolerance_timeout", tolerance_timeout_, 2.5);
        node_private.param("holonomic", holonomic_, false); //soru
        
        ros::NodeHandle node;
        odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PurePursuit::odomCallback, this, _1));
        vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        //path??
        path_sub_ = node.subscribe<nav_msgs::Path>("path", 1, boost::bind(&PurePursuit::pathCallback, this, _1));
        
        geometry_msgs::Twist currentVelocity_;
        ROS_DEBUG("Initialized");
    }
	
    //ok
    void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        boost::mutex::scoped_lock lock(odom_lock_);
        //ROS_INFO("odomcallback");
        base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
        base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
        base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
        currentVelocity_ = msg->twist.twist;
        ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        currentVelocity_.linear.x, currentVelocity_.linear.y, currentVelocity_.angular.z);
    }
	
    //ok
    void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(path_lock_);
        ROS_INFO("pathcallback");
        currentReferencePath_ = *msg;
        nextWayPoint_ = -1;
        for(int i = 0; i < currentReferencePath_.poses.size(); i++){
            ROS_INFO("Current Reference Path %d:\n",i);
            ROS_INFO("X: %lf\n",currentReferencePath_.poses[i].pose.position.x);    
            ROS_INFO("Y: %lf\n",currentReferencePath_.poses[i].pose.position.y);    
            ROS_INFO("Z: %lf\n",currentReferencePath_.poses[i].pose.position.z);
        }
    }
    
    //ok
    bool PurePursuit::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
        ROS_INFO("set_plan");
        current_waypoint_ = 0;
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
    geometry_msgs::PoseStamped PurePursuit::getCurrentPose(){
        geometry_msgs::PoseStamped pose, transformedPose;
        pose.header.frame_id = poseFrameId_;
        ROS_INFO("PP::getCurrent %f %f ==> %f,,,,,%f %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
        try {
            tfListener_.transformPose(pose.header.frame_id,
            pose, transformedPose);
            ROS_INFO("PP::TRY %f %f ==> %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        }
        catch (tf::TransformException& exception) {
            ROS_ERROR_STREAM("PurePursuit::getCurrentPose:ALOOOO " << 
            exception.what());
        }
        return transformedPose;
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
		//waypoint hangisi hangisi ? debug lazım gerek kalmamıs olabilir?
        tf::Stamped<tf::Pose> target_pose;
        tf::Stamped<tf::Pose> cur_pose_of_waypoints;
        tf::Stamped<tf::Pose> cur_pose_of_waypoints_next;
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], cur_pose_of_waypoints);
        //pose
        geometry_msgs::PoseStamped pose;
        //lookahead angle,distance: gerek kalmayabilir step icinde var..
        double lookAheadDistance_ = getLookAheadDistance(pose);
        double lookAheadAngle_ = getLookAheadAngle(pose);
        //arc distance
        double arcDistance=getArcDistance(lookAheadDistance_, lookAheadAngle_);
        
        //ROS_DEBUG("PP::CompuVelo current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
        //ROS_DEBUG("PP::CompuVelo target robot pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));

        //TODO:arcDistance to vel command nasil cikar?
        

        return true;
    }

    //ok
    double PurePursuit::getLookAheadThreshold(){
        return lookAheadRatio_*currentVelocity_.linear.x;
    }

    //ok cozuldu
    double PurePursuit::getLookAheadDistance(const geometry_msgs::PoseStamped& pose1){
        geometry_msgs::PoseStamped origin = getCurrentPose();
        geometry_msgs::PoseStamped transformedPose;
        
        try {
            tfListener_.transformPose(currentReferencePath_.header.frame_id,
            pose1, transformedPose);
        }
        catch (tf::TransformException& exception) {
            ROS_ERROR_STREAM("PurePursuit::getLookAheadDistance: " << 
            exception.what());
            return -1.0;
        }
        
        tf::Vector3 v1(origin.pose.position.x,origin.pose.position.y,origin.pose.position.z);
        tf::Vector3 v2(transformedPose.pose.position.x,transformedPose.pose.position.y,transformedPose.pose.position.z);
        return tf::tfDistance(v1, v2);
    }
	
    //ok sıkıntı olabilir, tf::poseda pose.position 
    double PurePursuit::getLookAheadAngle(const geometry_msgs::PoseStamped& pose1){
        geometry_msgs::PoseStamped origin = getCurrentPose();
        geometry_msgs::PoseStamped transformedPose;
        
        try {
            tfListener_.transformPose(currentReferencePath_.header.frame_id,
            pose1, transformedPose);
        }
        catch (tf::TransformException& exception) {
            ROS_ERROR_STREAM("PurePursuit::getLookAheadDistance: " << 
            exception.what());
            return -1.0;
        }
        
        tf::Vector3 v1(origin.pose.position.x,origin.pose.position.y,origin.pose.position.z);
        tf::Vector3 v2(transformedPose.pose.position.x,transformedPose.pose.position.y,transformedPose.pose.position.z);
        return tf::tfAngle(v1, v2);
    }
	
    //ok parametreler doubleda olablü bakmak lazım
    double PurePursuit::getArcDistance(double lookAheadDistance,double lookAheadAngle){
        if (std::abs(std::sin(lookAheadAngle)) >= epsilon_)
            return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle;
        else
            return lookAheadDistance;
    }

    //yeni eklendi
    int PurePursuit::getNextWayPoint(int wayPoint){ 
        ROS_INFO("PP::getnextwaypoint.start");  
        if (!currentReferencePath_.poses.empty()) {
            if (nextWayPoint_ >= 0) {
                geometry_msgs::PoseStamped origin = getCurrentPose();
                tf::Vector3 v_1(origin.pose.position.x,
                        origin.pose.position.y,
                        origin.pose.position.z);
                double lookAheadThreshold = getLookAheadThreshold();
                
                for (int i = nextWayPoint_; i < currentReferencePath_.poses.size();++i) {                
                    tf::Vector3 v_2(currentReferencePath_.poses[i].pose.position.x,
                          currentReferencePath_.poses[i].pose.position.y,
                          currentReferencePath_.poses[i].pose.position.z);
          
                    if (tf::tfDistance(v_1, v_2) > lookAheadThreshold)
                        return i;
                }

                return nextWayPoint_;
            }
            else
                return 0;
        }
        return -1;
    }

    //getinter
    bool PurePursuit::getInterpolatedPose(int wayPoint,
        geometry_msgs::PoseStamped& interpolatedPose){
            ROS_INFO("PP::getinterpolatedpose.start"); 
        if (!currentReferencePath_.poses.empty()) {
            if (wayPoint > 0) {
                double l_t = getLookAheadThreshold();
                double p_t = getLookAheadDistance(currentReferencePath_.poses[nextWayPoint_-1]);
        
                if (p_t < l_t) {
                    geometry_msgs::PoseStamped p_0 = getCurrentPose();
                    geometry_msgs::PoseStamped p_1 = 
                    currentReferencePath_.poses[wayPoint-1];
                    geometry_msgs::PoseStamped p_2 = 
                    currentReferencePath_.poses[wayPoint];
          
                    tf::Vector3 v_1(p_2.pose.position.x-p_0.pose.position.x,
                          p_2.pose.position.y-p_0.pose.position.y,
                          p_2.pose.position.z-p_0.pose.position.z);
                    tf::Vector3 v_2(p_1.pose.position.x-p_0.pose.position.x,
                          p_1.pose.position.y-p_0.pose.position.y,
                          p_1.pose.position.z-p_0.pose.position.z);
                    tf::Vector3 v_0(p_2.pose.position.x-p_1.pose.position.x,
                          p_2.pose.position.y-p_1.pose.position.y,
                          p_2.pose.position.z-p_1.pose.position.z);
          
                    double l_0 = v_0.length();
                    double l_1 = v_1.length();
                    double l_2 = v_2.length();
                    v_0.normalize();
                    v_2.normalize();
                    
                    double alpha_1 = M_PI-tf::tfAngle(v_0, v_2);
                    double beta_2 = asin(l_2*sin(alpha_1)/l_t);
                    double beta_0 = M_PI-alpha_1-beta_2;
                    double l_s = l_2*sin(beta_0)/sin(beta_2);
                    tf::Vector3 p_s(p_1.pose.position.x+v_0[0]*l_s,
                          p_1.pose.position.x+v_0[1]*l_s,
                          p_1.pose.position.x+v_0[2]*l_s);
                          
                    interpolatedPose.pose.position.x = p_s[0];
                    interpolatedPose.pose.position.y = p_s[1];
                    interpolatedPose.pose.position.z = p_s[2];
          
                    return true;
                }
            }
            interpolatedPose = currentReferencePath_.poses[wayPoint];
            return true;
        }
        return false;
    }

    //step
    bool PurePursuit::step(geometry_msgs::Twist& twist) {
        ROS_INFO("PP::step.start"); 
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
    
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
    
        nextWayPoint_ = getNextWayPoint(nextWayPoint_);

        if (nextWayPoint_ >= 0) {
            geometry_msgs::PoseStamped pose;
      
            if (getInterpolatedPose(nextWayPoint_, pose)) {
                double lookAheadDistance = getLookAheadDistance(pose);
                double lookAheadAngle = getLookAheadAngle(pose);

                double angularVelocity = 0.0;
                if (std::abs(std::sin(lookAheadAngle)) >= epsilon_) {
                    double radius = 0.5*(lookAheadDistance/std::sin(lookAheadAngle));
                    
                    double linearVelocity = velocity_;
                    if (std::abs(radius) >= epsilon_)
                        angularVelocity = linearVelocity/radius;

                    twist.linear.x = linearVelocity;
                    twist.angular.z = angularVelocity;
                    return true;
                }
            }
        }
        return false;
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
                ROS_INFO("glob transform");
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
