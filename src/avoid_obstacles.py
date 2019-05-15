#!/usr/bin/env python

import rospy
import message_filters
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, MapMetaData
import matplotlib.pyplot as plt

class Explorer:
    def __init__(self): #Class constructor
        rospy.init_node('obstacle_avoidance')  # Initiate a Node called 'obstacle_avoidance'
        laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)  # Subscribe to /scan topic to obtain laser info
        odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)  # Subscribe to /odom topic to obtain laser info
        map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)  # Subscribe to /map topic to obtain laser info

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # Create a publisher on the /cmd_vel topic

        self.move = Twist()
        self.laser = None
        self.odom = None
        self.map = None
        # actual drive state (initial: WallDetection)
        self.driveState = "WallDetection"

    #Function to read'nav_msgs/OccupancyGrid' messages and turn them into numpy arrays
    # from https://github.com/eric-wieser/ros_numpy
    def occupancygrid_to_numpy(self):
        data = numpy.asarray(self.map.data, dtype=numpy.int8).reshape(self.map.info.height, self.map.info.width)

        return numpy.ma.array(data, mask=data==-1, fill_value=-1)

    #Define a function called 'callback' for receiving parameters named 'msg'
    def laserscan_callback(self, laser_msg):
        self.laser = laser_msg

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def map_callback(self, map_msg):
        self.map = map_msg

    def startExplorer(self):
        while not rospy.is_shutdown():
            if (self.laser and self.odom):  # laser and odom data arrived from callback
                print('==========================================')
                print('s1 [270]') #value right-direction laser beam
                print self.laser.ranges[270]
                print('s2 [0]') #value front-direction laser beam
                print self.laser.ranges[0] #print the distance to an obstacle in front of the robot
                #The sensor returns a vector of 359 values being the initial value the corresponding to the front of the robot
                print('s3 [90]') #value left-direction laser beam
                print self.laser.ranges[90]
                print('Pose:')
                print self.odom.pose.pose

                # If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
                if self.laser.ranges[0] > 0.5:
                    self.move.linear.x = 0.22
                    self.move.angular.z = 0
                else:
                    self.move.linear.x = 0
                    self.move.angular.z = 1

                self.pub.publish(self.move)  # Publish movement

            if (self.map):  # occupancy data arrived from callback
                print('Map:')
                map=self.occupancygrid_to_numpy() #Occupancy map using numpy format
                #Plot occupancy map
                plt.imshow(map)
                plt.draw()
                plt.pause(0.001)
                plt.hold()  # toggle hold
                plt.hold(True)


        rospy.spin()

if __name__ == "__main__":
    explorer_obj = Explorer() #Create an object
    explorer_obj.startExplorer() #Call the function to initiate exploration
