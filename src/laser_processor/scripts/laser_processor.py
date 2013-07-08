#!/usr/bin/env python

# laser_processor.py
# last modified:
# Eric Kolker, 2013.07.08


import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Joy


def robot_driver():

	def command_drive(data):
		# initialize the message components
		header = Header()
		foo  = TwistWithCovarianceStamped()
		bar = TwistWithCovariance()
		baz = Twist()
		linear = Vector3()
		angular = Vector3()

		# get some data to publish
		# fake covariance until we know better
		covariance = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]
		# to be overwritten when we decide what we want to go where
		linear.x = data.axes[1] * 15
		linear.y = 0
		linear.z = 0
		angular.x = 0
		angular.y = 0
		angular.z = data.axes[0] * 10
		
		# put it all together
		# Twist
		baz.linear = linear
		baz.angular = angular
		
		# TwistWithCovariance
		bar.covariance = covariance
		bar.twist = baz

		# Header
		header.seq = data.header.seq
		header.frame_id = frame_id
		header.stamp = stopwatch.now()
		# seq = seq + 1
		# TwistWithCovarianceStamped
		foo.header = header
		foo.twist = bar

		# publish and log
		rospy.loginfo(foo)
		pub.publish(foo)


	def command_lidar(data):
		how_much_rotation = Vector3()
		how_much_rotation.x = 0
		how_much_rotation.z = 0

		# right stick up/down
		foo = data.axes[4]
		# take out some noise
		foo = round(foo, 1)
		how_much_rotation.y = foo

		# publish and log
		rospy.loginfo(how_much_rotation)
		firin_mah_lazer.publish(how_much_rotation)


	def callback(data):
		command_drive(data)
		command_lidar(data)


	# overhead for the Headers
	frame_id = "robot frame"
	stopwatch = rospy.Time()

	# intiialize the publishers
	pub = rospy.Publisher('gaia_gazebo_plugin/gaia_driver', TwistWithCovarianceStamped)
	firin_mah_lazer = rospy.Publisher('gaia_gazebo_plugin/gaia_laser_command', Vector3)

	# initalize the node
	rospy.init_node('gaia_simple_driver')

	# init a listener for the joystick
	sub = rospy.Subscriber("joy", Joy, callback)
	print "made a subscriber"

	# wait for input from the joystick
	rospy.spin()
	print "i spun"





if __name__ == '__main__':
	try:
		robot_driver()
	except rospy.ROSInterruptException:
		pass
