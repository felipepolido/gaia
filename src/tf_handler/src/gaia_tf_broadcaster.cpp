#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>

std::string robot_name;



void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w) );

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", robot_name));

  tf::TransformBroadcaster chassi_lidar_br;

  tf::Transform chassi_lidar;
  chassi_lidar.setOrigin( tf::Vector3(1.2, -0.3, 0.35) );
  chassi_lidar.setRotation( tf::Quaternion(0, 0, 0, 1) );

  chassi_lidar_br.sendTransform(tf::StampedTransform(chassi_lidar, ros::Time::now(), robot_name, "lidar"));


}

int main(int argc, char **argv){
  ros::init(argc, argv, "gaia_tf_broadcaster");

  robot_name = "base_link";

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gaia_gazebo_plugin/odom", 10, &poseCallback);


  

  ros::spin();
  return 0;
}