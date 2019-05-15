#!/usr/bin/env python

import rospy
import message_filters
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, MapMetaData
import matplotlib.pyplot as plt


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
    print laserscan.ranges[90]
    print('Pose:')
    print odometry.pose.pose


    #If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
    if laserscan.ranges[0]>0.5:
        move.linear.x=0.22
        move.angular.z=0
    else:
        move.linear.x=0
        move.angular.z=1

    pub.publish(move) #Publish movement

#Function to read'nav_msgs/OccupancyGrid' messages and turn them into numpy arrays
# from https://github.com/eric-wieser/ros_numpy
def occupancygrid_to_numpy(msg):
	data = numpy.asarray(msg.data, dtype=numpy.int8).reshape(msg.info.height, msg.info.width)

	return numpy.ma.array(data, mask=data==-1, fill_value=-1)

#Callback function to read map from rsviz
def callback2(occupancy):
    print('Map:')
    map = occupancygrid_to_numpy(occupancy)
    plt.imshow(map)
    plt.draw()
    plt.pause(0.001)
    plt.hold()  # toggle hold
    plt.hold(True)

rospy.init_node('obstacle_avoidance') #Initiate a Node called 'obstacle_avoidance'
laser_sub = message_filters.Subscriber('/scan', LaserScan) #Subscribe to /scan topic to obtain laser info
odom_sub = message_filters.Subscriber('/odom', Odometry) #Subscribe to /odom topic to obtain laser info
map_sub = message_filters.Subscriber('/map', OccupancyGrid) #Subscribe to /map topic to obtain laser info

ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odom_sub], 1, 1)
ts.registerCallback(callback)

ts2 = message_filters.ApproximateTimeSynchronizer([map_sub], 1, 1)
ts2.registerCallback(callback2)

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) #Create a publisher on the /cmd_vel topic
move=Twist()

rospy.spin()
