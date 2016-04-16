#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from icp_test.srv import *

from cython_catkin_example import cython_catkin_example

import math
import numpy as np
import pylab as plt
import itertools
# import curses
import matplotlib.animation as animation

from scipy.spatial import KDTree

import IcpTestTools

from optparse import OptionParser
import os, sys

rospy.init_node('local_3d_map_server_node', anonymous=False)
g_pub_map = rospy.Publisher("icp_globalmap", PointCloud2, latch = True)

opt_parser = OptionParser()
opt_parser.add_option('-f','--filename', dest='filename', type='string')
opts, args = opt_parser.parse_args(sys.argv[1:])

def getCloudFromFile(filename):
	return IcpTestTools.getFloatNumberFromReadLines(open(filename,'r'),3)

def talker():
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'velodyne' #'map'
	wmap = getCloudFromFile(opts.filename)
	g_pcloud = pc2.create_cloud_xyz32(header, wmap)
	g_pub_map.publish(g_pcloud)
	rospy.spin()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
