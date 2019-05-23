#!/usr/bin/env python

import rospy
from threading import Thread, Lock
import message_filters
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, MapMetaData
import math
import time
import matplotlib.pyplot as plt


class Explorer:
    def __init__(self): #Class constructor
        rospy.init_node('obstacle_avoidance')  # Initiate a Node called 'obstacle_avoidance'
        laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)  # Subscribe to /scan topic to obtain laser info
        odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)  # Subscribe to /odom topic to obtain laser info
        map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)  # Subscribe to /map topic to obtain laser info

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  # Create a publisher on the /cmd_vel topic

        #Set up initial values
        self.move = Twist()
        self.laser = None #Data from the laserscan
        self.odom = None #Data from the odometry
        self.map = None #Data from the map in rviz
        self.quat = None #Quaternions
        self.ea = None #Euler angles
        self.starting = None #Starting orientation

        # Set up wall detection
        self.mutex2 = Lock() #Mutual exclusion
        self.mindist = None #Front distance
        self.min_index = None #Position of the minimum front distance in an array
        self.latdist = None #Lateral distance

        # Set up wall follower
        self.minwalldist = 0.3 #Minimum lateral distance
        self.maxwalldist = 0.35 #Maximum lateral distance
        self.turnSpeed = -0.3 #Default turn speed (turns right)
        self.distanceToWall = 0.6  # minimum front distance for obstacles detection
        self.angle = 1.57  # around 90 degree
        self.mutex = Lock() #Mutual exclusion

        # Current drive state (initial: WallDetection)
        self.state = "WallDetection"


    #Function to read'nav_msgs/OccupancyGrid' messages and turn them into numpy arrays
    # from https://github.com/eric-wieser/ros_numpy
    def occupancygrid_to_numpy(self):
        data = numpy.asarray(self.map.data, dtype=numpy.int8).reshape(self.map.info.height, self.map.info.width)

        return numpy.ma.array(data, mask=data==-1, fill_value=-1)

    #Define a function called 'callback' for receiving parameters named 'msg'
    #Laserscan callback
    def laserscan_callback(self, laser_msg):
        self.laser = laser_msg #Store in a global variable last reading

    # Odometry callback
    def odom_callback(self, odom_msg):
        self.odom = odom_msg #Store in a global variable last reading

    # Map callback
    def map_callback(self, map_msg):
        self.map = map_msg #Store in a global variable last reading

    #Main function of the program
    def startExplorer(self):
        while not rospy.is_shutdown():
            if (self.laser and self.odom):  # laser and odom data arrived from callback

                #Print information about the state
                print('==========================================')
                print('s1 [270]') #value right-direction laser beam
                print self.laser.ranges[270]
                print('s2 [0]') #value front-direction laser beam. values at 0 degree
                print self.laser.ranges[0] #print the distance to an obstacle in front of the robot
                #The sensor returns a vector of 359 values being the initial value the corresponding to the front of the robot
                print('s3 [90]') #value left-direction laser beam
                print self.laser.ranges[90]
                print('s4 [45:90]')  # value left-direction laser beam
                print min(self.laser.ranges[45:90])
                print('Pose:') #Pose of the robot
                print self.odom.pose.pose
                print ('State: ') #Current state of the robot
                print self.state


                #Save the quaternion orientation
                self.quat= numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                self.ea = self.quaternion_to_euler() #Convert the Quaternion to Euler angles

                #Different states to detect, aproximate and follow walls
                if (self.state == "WallDetection"):
                    #Detect wall
                    self.wallDetection()
                elif (self.state == "DriveToWall"):
                    #Go to the wall
                    if (self.mutex.locked() == False): #The lock is released
                        # Obstacle in front of the robot or wall arrived
                        self.mindist = self.laser.ranges[0]

                        if (self.mindist <= self.distanceToWall): #You detect an obstacle
                            # save the actual position used for loop detection
                            self.move.linear.x = 0.0
                            self.move.angular.z = 0.0
                            self.pub.publish(self.move)  # Publish movement
                            self.state = "WallFollow"
                            #Last position update
                            self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                            self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

                            self.currentorient()

                        else:
                            #Move straight forward
                            self.move.linear.x = 0.3
                            self.move.angular.z = 0.0
                            self.pub.publish(self.move)  # Publish movement

                elif (self.state == "WallFollow"):
                    #Follow the wall
                    self.wallFollow()

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

    # Detect your starting orientation
    # --> Right 1
    # <--Left 2
    # UP 3
    # DOWN 4
    def currentorient(self):
        self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                 self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
        self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles
        if (math.fabs(self.ea[2]) > 70) and (math.fabs(self.ea[2]) < 110) and (self.ea[2] < 0):
            self.starting = 2  # Left
        elif (math.fabs(self.ea[2]) > 70) and (math.fabs(self.ea[2]) < 110) and (self.ea[2] > 0):
            self.starting = 1  # Right
        elif (math.fabs(self.ea[2]) < 20):
            self.starting = 4  # Down
        elif (math.fabs(self.ea[2]) > 160):
            self.starting = 3  # Up
        print("Starting point: ")
        print self.starting

    #Function to transform quaternion angles to euler angles
    #From https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr?rq=1
    def quaternion_to_euler(self):
        x=self.quat[0]
        y=self.quat[1]
        z=self.quat[2]
        w=self.quat[3]

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        #Roll, pitch and yaw
        return X, Y, Z

    #Function to detect the wall with minimum distance in front of the robot
    def wallDetection(self):

        self.mutex2.acquire() #Block other threads. One thread running at a time
        try:
        # search for a wall to follow
            #Find minimum distance to a wall
            self.mindist=min(self.laser.ranges[0:20])
            print ('Min distance is: ')
            print self.mindist
            if (self.mindist<1.5): #Min distance to an obstacle less than 1.5
                self.min_index=numpy.argmin(self.laser.ranges[0:20]) #Min distance position
                print('Rotating radians')
                print self.min_index*3.1415*2/360

                #Rotate that distance
                self.rotate_angle(self.min_index*3.1415*2/360)
                self.state = "DriveToWall"
            else:
                self.move.linear.x = 0.22 #Move forward
                self.pub.publish(self.move)  # Publish movement
                # wait until the robot has stopped the rotation
                rospy.sleep(3)

        finally:
            self.mutex2.release() #Release the lock


    #Function to follow the wall you have detected
    def wallFollow(self):

        #We are following left walls
        #Save last position data
        self.mindist = self.laser.ranges[0]
        self.latdist =  min(self.laser.ranges[45:90])
        self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
        self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

        #Print some data of the position
        print('Orientation in z euler angle: ')
        print self.ea[2]
        print ("Lateral distance:")
        print self.latdist
        print ("Initial State")
        print self.starting

        #Check in every instant that there is an obstacle in the left, if not stop the robot and check lasers
        if (self.latdist>=1) and min(self.laser.ranges[100:140])<0.7: #No obstacle at the left, but an obstacle in the left bottom part, rotate to follow the wall

            # Move forward 2 seconds
            self.move.linear.x = 0.2
            self.move.angular.z = 0.0
            self.pub.publish(self.move)
            time.sleep(2)

            #Rotate a bit more than 90 degrees
            self.turnSpeed = 0.3
            self.rotate_angle(1.70)

            #Move forward 6 seconds
            self.move.linear.x = 0.2
            self.move.angular.z = 0.0
            self.pub.publish(self.move)
            time.sleep(6)

            #Check orientation
            self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                     self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
            self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

            self.currentorient()

        elif (self.latdist>=1) and min(self.laser.ranges[100:140])>0.7: #No lateral distance and no object in the left bottom part, you are lost
            print("You are lost")
            self.state = "WallDetection"

        elif (self.latdist< self.minwalldist): #Too close to a lateral wall

            #Rotate and move linearly 1 second
            self.turnSpeed = -0.3
            self.rotate_angle(0.087)
            self.move.linear.x = 0.3
            self.move.angular.z = 0.0
            self.pub.publish(self.move)
            time.sleep(1)

        # Starting orientation UP
        if (self.starting == 3): #Obstacle in front of the robot, rotate 90 degrees

            #Check obstalces in front of the robot
            if (self.mindist < self.distanceToWall): #Obstacle in front of the robot, rotate
                self.move.linear.x = 0
                self.move.angular.z = 0.0
                self.pub.publish(self.move)
                self.turnSpeed = -0.3
                self.rotate_angle(1.57) #Rotate 90 degrees
                self.starting = 1

                #Check the rotation (Added to solve some extrange behaviours of the robot)
                self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                         self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

                #If the orientation angle is less than 85 degrees, rotate 5 degrees more
                if (self.ea[2] < 85):
                    self.turnSpeed = -0.3
                    self.rotate_angle(0.087)

                time.sleep(0.01)

            elif (self.ea[2] < 175) and (self.ea[2] > 0): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = 0.3
                self.rotate_angle(0.087)

            elif (self.ea[2] > -175) and (self.ea[2] < 0): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = -0.3
                self.rotate_angle(0.087)

            else: #Move straight forward
                self.move.linear.x = 0.3
                self.move.angular.z = 0.0

        #Starting orientation LEFT
        elif (self.starting == 2): #Obstacle in front of the robot, rotate 90 degrees
            if (self.mindist < self.distanceToWall):
                self.move.linear.x=0
                self.move.angular.z = 0.0
                self.pub.publish(self.move)
                self.turnSpeed = -0.3
                self.rotate_angle(1.57)
                self.starting = 3

                #Check the rotation (Added to solve some extrange behaviours of the robot)
                self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                         self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

                if (math.fabs(self.ea[2]) < 175):
                    self.turnSpeed = -0.3
                    self.rotate_angle(0.087)

                time.sleep(0.01)

            elif (math.fabs(self.ea[2]) < 85): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = -0.3
                self.rotate_angle(0.087)

            elif (math.fabs(self.ea[2]) > 95): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = 0.3
                self.rotate_angle(0.087)

            # Move straight forward
            else:
                self.move.linear.x = 0.3
                self.move.angular.z = 0.0

        # Starting orientation DOWN
        elif (self.starting ==4): #Obstacle in front of the robot, rotate 90 degrees
            if (self.mindist < self.distanceToWall):
                self.move.linear.x=0
                self.move.angular.z = 0.0
                self.pub.publish(self.move)
                self.turnSpeed = -0.3
                self.rotate_angle(1.57)
                self.starting = 2

                #Check the rotation (Added to solve some extrange behaviours of the robot)
                self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                         self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

                if (math.fabs(self.ea[2]) < 85):
                    self.turnSpeed = -0.3
                    self.rotate_angle(0.087)

                time.sleep(0.01)

            elif (self.ea[2] > 5) and (self.ea[2] > 0): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = -0.3
                self.rotate_angle(0.087)

            elif (self.ea[2] < -5): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = 0.3
                self.rotate_angle(0.087)

            # Move straight forward
            else:
                self.move.linear.x = 0.3
                self.move.angular.z = 0.0

        # Starting orientation RIGHT
        elif (self.starting ==1): #Obstacle in front of the robot, rotate 90 degrees
            if (self.mindist < self.distanceToWall):
                self.move.linear.x=0
                self.move.angular.z = 0.0
                self.pub.publish(self.move)
                self.turnSpeed = -0.3
                self.rotate_angle(1.90)
                self.starting = 4

                #Check the rotation (Added to solve some extrange behaviours of the robot)
                self.quat = numpy.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                         self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
                self.ea = self.quaternion_to_euler()  # Convert the Quaternion to Euler angles

                if (math.fabs(self.ea[2]) > 5):
                    self.turnSpeed = -0.3
                    self.rotate_angle(0.087)

                print ("Change to state 1 from 3, go right")
                time.sleep(0.01)

            elif (self.ea[2] < 85): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = 0.3
                self.rotate_angle(0.087)

            elif (self.ea[2] > 95): #If the angle in that direction is not in a range, rotate
                self.turnSpeed = -0.3
                self.rotate_angle(0.087)


            else: # Move straight forward
                self.move.linear.x = 0.3
                self.move.angular.z = 0.0

        self.pub.publish(self.move)  # Publish movement

    #Simple rotation function by the relative angle in rad
    #From https://github.com/g40st/ROS_maze_challenge/blob/master/ch_171744_maze/src/maze.py
    def rotate_angle(self, angle):
        self.mutex.acquire() #Block other threads. One thread running at a time
        try:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            self.pub.publish(self.move)

            self.move.linear.x = 0.0
            self.move.angular.z = self.turnSpeed
            # setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_angle = 0

            while (current_angle <= angle):
                self.pub.publish(self.move)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(self.turnSpeed) * (t1 - t0)

            # Forcing the robot to stop after rotation
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            self.pub.publish(self.move)
            time.sleep(0.01)

        finally:
            self.mutex.release() #Release the lock

if __name__ == "__main__":
    explorer_obj = Explorer() #Create an object
    explorer_obj.startExplorer() #Call the function to initiate exploration
