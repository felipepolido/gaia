#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>

std::string robot_name;



void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.z, 0, 0,0) );

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", robot_name));

/*
  static tf::TransformBroadcaster br2;

  tf::Transform transform2;
  transform2.setOrigin( tf::Vector3(0.0,0.0,0.0) );
  transform2.setRotation( tf::Quaternion(0,0,0,0) );

  br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "map"));
*/
  

}

int main(void){
  int argc = 0;
  ros::init(argc, NULL, "gaia_tf_broadcaster");

  robot_name = "gaia";

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gaia_gazebo_plugin/odom", 10, &poseCallback);


  ros::spin();
  return 0;
};