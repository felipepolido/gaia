#!/usr/bin/env python

# laser_processor.py
# last modified:
# Eric Kolker, 2013.07.08


import rospy
# from geometry_msgs.msg import TwistWithCovarianceStamped
# from geometry_msgs.msg import TwistWithCovariance
# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Header
# from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState





def callback(data):
	pass



def laser_processor():

	# overhead for the Headers
	stopwatch = rospy.Time()

	# intiialize the publisher
	laser_publisher = rospy.Publisher('processed_laser', LaserScan)

	# initalize the node
	rospy.init_node('laser_processor')

	# init a listener for the joystick
	# sub = rospy.Subscriber("joy", Joy, callback)
	


	print "Firin' mah lazer!"
	rospy.spin()
	print "Your laser overheated and must recharge"



if __name__ == '__main__':
	try:
		laser_processor()
	except rospy.ROSInterruptException:
		pass
