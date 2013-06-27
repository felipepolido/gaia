#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point


def robot_driver():
    pub = rospy.Publisher('gaia_gazebo_plugin/gaia_driver', Point)
    rospy.init_node('gaia_simple_driver')
    while not rospy.is_shutdown():
        foo  = Point()
        foo.x = -100;#"hello world %s" % rospy.get_time()
        foo.y = 100;
        foo.z = 0;
        
        rospy.loginfo(foo)
        pub.publish(foo)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        robot_driver()
    except rospy.ROSInterruptException:
        pass
