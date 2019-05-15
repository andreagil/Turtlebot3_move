#!/usr/bin/env python

import rospy
import message_filters
import numpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
#from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt
#import sys
#from numpy.lib.stride_tricks import as_strided

#global order
#order=[0]

#Define a function called 'callback' that receives a parameter named 'msg'
def callback(occupancy):
    print('Map:')
    map = occupancygrid_to_numpy(occupancy)
    print map
    plt.imshow(map)
    plt.show()

def occupancygrid_to_numpy(msg):
	data = numpy.asarray(msg.data, dtype=numpy.int8).reshape(msg.info.height, msg.info.width)

	return numpy.ma.array(data, mask=data==-1, fill_value=-1)


rospy.init_node('state_map') #Initiate a Node called 'state_map'
map_sub = message_filters.Subscriber('/map', OccupancyGrid)
ts = message_filters.ApproximateTimeSynchronizer([map_sub], 1, 1)
ts.registerCallback(callback)

rospy.spin()
