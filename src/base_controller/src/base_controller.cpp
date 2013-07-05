#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

std::string topic_name;

geometry_msgs::Twist current_cmd_vel;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{

  current_cmd_vel = *msg;
  //Receive latest command velocity from navigation package

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}



int main(int argc, char **argv)
{

  topic_name = "base_controller";

  ros::init(argc, argv, "base_controller");


  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmd_vel_callback);

  ros::Publisher pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/gaia_gazebo_plugin/gaia_driver", 100);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    ros::spinOnce();
    //Publish Base driving commands at 10Hz

    ros::Time current_time = ros::Time::now();
    geometry_msgs::TwistWithCovarianceStamped base_driver;

    base_driver.header.frame_id = topic_name;
    base_driver.header.stamp.sec = current_time.sec;
    base_driver.header.stamp.nsec = current_time.nsec;
    //base_driver.child_frame_id = "/";


    base_driver.twist.twist = current_cmd_vel;

    double temp_covariance[36] = {0};
    for(unsigned int t = 0; t < 6 ; t++)
    {
      base_driver.twist.covariance[t*6] = 1;
    }

    pub.publish(base_driver);



    loop_rate.sleep();
    ++count;
  }

  return 0;
}

