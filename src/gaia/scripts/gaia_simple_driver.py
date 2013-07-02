#!/usr/bin/env python

# gaia_simple_driver.py
# last modified:
# Eric Kolker, 2013.07.01


import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3



def robot_driver():

	# intiialize the publisher
	pub = rospy.Publisher('gaia_gazebo_plugin/gaia_driver', TwistWithCovarianceStamped)

	# initalize the node
	rospy.init_node('gaia_simple_driver')

	while not rospy.is_shutdown():

		# make a header
		header = String(data = str(rospy.get_time()))

		# initialize the message components
		foo  = TwistWithCovarianceStamped()
		bar = TwistWithCovariance()
		baz = Twist()
		linear = Vector3()
		angular = Vector3()

		# get some data to publish
		# fake covariance until we know better
		covariance = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]
		# to be overwritten when we decide what we want to go where
		linear.x = 1
		linear.y = 0
		linear.z = 0
		angular.x = 0
		angular.y = 0
		angular.z = 1
		

		# put it all together
		baz.linear = linear
		baz.angular = angular
		
		bar.covariance = covariance
		bar.twist = baz

		foo.header = header
		foo.twist = bar

		# publish and log
		rospy.loginfo(foo)
		pub.publish(foo)
		rospy.sleep(1.0)


if __name__ == '__main__':
	try:
		robot_driver()
	except rospy.ROSInterruptException:
		pass
