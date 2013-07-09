#!/usr/bin/env python

# laser_processor.py
# last modified:
# Eric Kolker, 2013.07.08


import rospy

# from geometry_msgs.msg import TwistWithCovarianceStamped
# from geometry_msgs.msg import TwistWithCovariance
# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

from std_msgs.msg import String
from std_msgs.msg import Header

# from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState

from nav_msgs.msg import Odometry






def gazebo_laser_callback(data, args):
	'''
	Callback for the gazebo-based (sim) laser found on the front of the robot.

	args: list of things! current contents:
		0	publisher to use
		1	pose of scanner wrt the world

	'''
	




def laser_processor():

	# overhead for the Headers
	stopwatch = rospy.Time()

	# intiialize the publisher
	laser_publisher = rospy.Publisher('processed_laser', LaserScan)

	# initalize the node
	rospy.init_node('laser_processor')


	# 	set up the listener for raw laserscans

	# for now we assume just the publisher is needed as an arg, but more soon?
	# todo: get lidar pose wrt the world (map?)
	gazebo_scan_callback_args = [laser_publisher]

	# init a listener for the gazebo laser
	gazebo_laser_sub = rospy.Subscriber("/gaia_gazebo_plugin/base_scan", \
		LaserScan, gazebo_laser_callback, gazebo_scan_callback_args)

	gazebo_pose_sub = rospy.Subscriber("/gaia_gazebo_plugin/odom", \

	

	print "Firin' mah lazer!"
	rospy.spin()
	print "Your laser overheated and must recharge"



if __name__ == '__main__':
	try:
		laser_processor()
	except rospy.ROSInterruptException:
		pass
