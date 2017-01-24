#include <string>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

void mycallback(const nav_msgs::Odometry::ConstPtr& msg){
tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
tf::Matrix3x3 m(q);
double roll,pitch,yaw;
m.getRPY(roll,pitch,yaw);
 ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
ROS_INFO("Yaw,Pitch,Roll->y:[%f],p:[%f],r:[%f]",yaw,pitch,roll);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

int main(int argc,char **argv){
ros::init(argc,argv,"hop");
ros::NodeHandle nh;
ros::Subscriber sub=nh.subscribe("/mobile_base_controller/odom",1000,mycallback);
ros::spin();
return 0;
}


