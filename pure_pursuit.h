#ifndef POSE_FOLLOWER_POSE_FOLLOWER_H_
#define POSE_FOLLOWER_POSE_FOLLOWER_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/trajectory_planner_ros.h>


namespace pure_pursuit {
  class PurePursuit : public nav_core::BaseLocalPlanner {
    public:
      PurePursuit();
      //init ?
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      //is goal reached ++
      bool isGoalReached();
      //set plan global -> global plan aliyor
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      //compute velocity commands ++
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	
	private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }
      //difference 2d ??
      geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose&  pose2);
      //limit twist ??
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      //heading difference ??
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);
      //transform global plan ??
	  bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);
	  //odomcallback def ++
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      //stopped ??
      bool stopped();
      
      tf::TransformListener* tf_;
      
      costmap_2d::Costmap2DROS* costmap_ros_;
      
      ros::Publisher vel_pub_;
      
      double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;

      // nlimpert - 2015-04-10: lookahead tweak
      int lookahead_count_;
      double lookahead_weight_;
      
      
      boost::mutex odom_lock_;
      ros::Subscriber odom_sub_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
      int samples_;
  };
};
#endif
