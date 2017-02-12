#ifndef PURE_PURSUIT_PURE_PURSUIT_H_
#define PURE_PURSUIT_PURE_PURSUIT_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/trajectory_planner_ros.h>

namespace pure_pursuit {
    class PurePursuit : public nav_core::BaseLocalPlanner {
        public:
            PurePursuit();
            void initialize(std::string name, tf::TransformListener* tf,costmap_2d::Costmap2DROS* costmap_ros);
            bool isGoalReached();
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            //int getNextWayPoint(int wayPoint);
            bool getInterpolatedPose(int wayPoint,geometry_msgs::PoseStamped& interpolatedPose);
            //bool step(geometry_msgs::Twist& twist);
            //bool sendCmdVel(geometry_msgs::Twist& cmd_vel);
            double getLookAheadThreshold();
            double getLookAheadDistance(int waypoint);
            double getLookAheadAngle(int waypoint);
            double getArcDistance(int waypoint);
            int getClosestWP();
            int getNextWP();
            
        private:
            inline double sign(double n){
                return n < 0.0 ? -1.0 : 1.0;
            }
        
        geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
        geometry_msgs::PoseStamped getCurrentPose();
        bool transformGlobalPlan(const tf::TransformListener& tf, 
            const std::vector<geometry_msgs::PoseStamped>& global_plan, 
            const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
            std::vector<geometry_msgs::PoseStamped>& transformed_plan);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        bool stopped();
        double headingDiff(double x, double y, double pt_x, double pt_y, double heading);
        geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose& pose2);
      
        tf::TransformListener* tf_;
        tf::TransformListener tfListener_;
      
        costmap_2d::Costmap2DROS* costmap_ros_;
      
        ros::Publisher vel_pub_;
      
        double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;

        // nlimpert - 2015-04-10: lookahead tweak
        int lookahead_count_;
        double lookahead_weight_;
        double tolerance_timeout_;
        double max_vel_lin_, max_vel_th_;
        double min_vel_lin_, min_vel_th_;
        double min_in_place_vel_th_, in_place_trans_vel_;
        bool holonomic_;
        
        boost::mutex odom_lock_;
        ros::Subscriber odom_sub_;
        nav_msgs::Odometry base_odom_;

        //boost::mutex path_lock_;
        //ros::Subscriber path_sub_;
        //nav_msgs::Path currentReferencePath_;
        
        geometry_msgs::Twist currentVelocity_;
        std::string poseFrameId_;

        
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        base_local_planner::TrajectoryPlannerROS collision_planner_;
        ros::Time goal_reached_time_;
        
        double max_heading_diff_before_moving_;
        bool turn_in_place_first_,allow_backwards_;
        double arcDistance_;
        double trans_stopped_velocity_, rot_stopped_velocity_;
        unsigned int current_waypoint_; 
        int samples_,nextWayPoint_,initialWayPoint_;
        double velocity_,epsilon_,frequency_,lookAheadRatio_;
    };
};
#endif
