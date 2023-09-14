#!/usr/bin/env python
import numpy as np
import rospy
import sys


from turtle_patrol.srv import MyPatrol  # Import service type


def patrol_client(name):
    # Initialize the client node
    rospy.init_node('turtle1_patrol_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/turtle1/patrol')
    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy(
            '/turtle1/patrol', MyPatrol)
        vel = 2.0  # Linear velocity
        omega = 1.0  # Angular velocity
        x = 2.0
        y = 3.0
        theta = 5.0
        rospy.loginfo('Command turtle1 to patrol')
        # Call patrol service via the proxy
        patrol_proxy(vel, omega, x, y, theta, name)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    turtle_name = sys.argv[6]
    patrol_client(turtle_name)
