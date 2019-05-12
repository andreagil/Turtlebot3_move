#!/usr/bin/env python

import rospy
import message_filters
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from rospy.numpy_msg import numpy_msg

#global order
#order=[0]

#Define a function called 'callback' that receives a parameter named 'msg'
def callback(laserscan, odometry):
    #global order
    print('==========================================')
    print('s1 [270]') #value right-direction laser beam
    print laserscan.ranges[270]
    print('s2 [0]') #value front-direction laser beam
    print laserscan.ranges[0] #print the distance to an obstacle in front of the robot
    #The sensor returns a vector of 359 values being the initial value the corresponding to the front of the robot
    print('s3 [90]') #value left-direction laser beam
    print laserscan.ranges[270]
    print('Pose:')
    print odometry.pose.pose
    #If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
    if laserscan.ranges[0]>0.5:
        move.linear.x=0.22
        move.angular.z=0
    else:
        move.linear.x=0
        move.angular.z=1

    """var=0
    var=msg.ranges[0]
    order = numpy.append(order, var)

    print order"""
    pub.publish(move)


rospy.init_node('obstacle_avoidance') #Initiate a Node called 'obstacle_avoidance'
laser_sub = message_filters.Subscriber('/scan', LaserScan)
odom_sub = message_filters.Subscriber('/odom', Odometry)
ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odom_sub], 1, 1)
ts.registerCallback(callback)
pub = rospy.Publisher('cmd_vel', Twist) #Create a publisher on the /cmd_vel topic
move=Twist()

rospy.spin()
