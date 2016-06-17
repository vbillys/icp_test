#! /usr/bin/env python

import IcpTestTools
import ujson
import jsonpickle

import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
import sensor_msgs.point_cloud2 as pc2
import tf
import math

import sys
from optparse import OptionParser
import os
import errno

import copy
import zlib
import string, StringIO
import pcl
import numpy as np

opt_parser = OptionParser()
opt_parser.add_option('-f','--contfile', dest='filename', type='string', default=None)
opts, args = opt_parser.parse_args(sys.argv[1:])

if opts.filename is None:
	raise Exception('no file is given')
	exit()

def getCloudFromPCD(ffname):
	pcl_cloud = pcl.load(ffname)
	return pcl_cloud.to_array()

rospy.init_node('local_3d_map_server_node', anonymous=False)
g_pub_map = rospy.Publisher("icp_globalmap", PointCloud2, latch = True)
header = Header()
header.stamp = rospy.Time.now()
# header.frame_id = 'velodyne' #'map'
header.frame_id = 'world' #'map'
wmap = getCloudFromPCD(opts.filename)
g_pcloud = pc2.create_cloud_xyz32(header, wmap)
g_pub_map.publish(g_pcloud)
rospy.spin()
