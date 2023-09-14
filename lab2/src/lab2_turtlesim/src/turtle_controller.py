#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy
import sys

# Import the String message type from the /msg directory of the std_msgs package.
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# Define the method which contains the node's main functionality
def talker(turtle_name):

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /chatter_talk
    my_topic = turtle_name + '/cmd_vel'
    pub = rospy.Publisher(my_topic, Twist, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)
        # pub_string = "hello world %s" % (rospy.get_time())

        twist = Twist()
        control = input("enter random key for now:\n")
        twist.angular.z = 0
        if (control == 'a'):
            twist.linear.x = -2
            twist.linear.y = 0
        elif(control == 's'):
            twist.linear.x = 0
            twist.linear.y = -2            
        elif(control == 'd'):
            twist.linear.x = 2
            twist.linear.y = 0
        elif(control == 'w'):
            twist.linear.x = 0
            twist.linear.y = 2
    
        
        # Publish our string to the 'chatter_talk' topic
        pub.publish(twist)
        #print(rospy.get_name() + ". " +  twist)
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    turtle_name = sys.argv[1]

    try:
        talker(turtle_name)
    except rospy.ROSInterruptException: pass