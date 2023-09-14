#!/usr/bin/env python
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import rospy
from std_srvs.srv import Empty
from turtle_patrol.srv import MyPatrol  # Service type
from turtlesim.srv import TeleportAbsolute


def patrol_callback(request):
    rospy.wait_for_service('clear')
    rospy.wait_for_service('/turtle1/teleport_absolute')
    clear_proxy = rospy.ServiceProxy('clear', Empty)
    teleport_proxy = rospy.ServiceProxy(
        '/turtle1/teleport_absolute',
        TeleportAbsolute
    )

    vel = request.vel  # Linear velocity
    omega = request.omega  # Angular velocity
    x = request.x
    y = request.y
    theta = request.theta
    name = request.name

    topic = "/"+name+"/cmd_vel"
    pub = rospy.Publisher(
        topic, Twist, queue_size=50)
    
    cmd = Twist()
    cmd.linear.x = vel
    cmd.angular.z = omega

    topicPose = "/"+name+"/pose"
    pub2 = rospy.Publisher(
        topicPose, Pose, queue_size=50)

    pose = Pose()
    pose.x = x
    pose.y = y
    pose.theta = theta

    
    # Publish to cmd_vel at 5 Hz
    rate = rospy.Rate(5)
    # Teleport to initial pose
    teleport_proxy(9, 5, np.pi/2)

    # Clear historical path traces
    clear_proxy()
    while not rospy.is_shutdown():
        pub.publish(cmd)  # Publish to cmd_vel
        pub2.publish(pose) # Publish to pose
        rate.sleep()  # Sleep until 

    return cmd  # This line will never be reached

def patrol_server():
    # Initialize the server node for turtle1
    rospy.init_node('turtle1_patrol_server')
    # Register service
    rospy.Service(
        '/turtle1/patrol',  # Service name
        MyPatrol,  # Service type
        patrol_callback  # Service callback
    )
    rospy.loginfo('Running patrol server...')
    rospy.spin() # Spin the node until Ctrl-C


if __name__ == '__main__':
    patrol_server()

