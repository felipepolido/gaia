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
        header = str(rospy.get_time())

        # initialize the message components
        foo  = TwistWithCovarianceStamped()
        bar = TwistWithCovariance()
        baz = Twist()
        # fake covariance until we know better
        qux = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]
        quxx = []
        corge = []

        # put it all together
        foo.header = header
        bar.covariance = qux
        baz.libne
        foo.twist = bar


        
        rospy.loginfo(foo)
        pub.publish(foo)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        robot_driver()
    except rospy.ROSInterruptException:
        pass
