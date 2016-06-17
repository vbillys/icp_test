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
opt_parser.add_option('-s','--startframe', dest='start_frame_no', type='int', default=1)
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
# opt_parser.add_option('-f','--contfile', dest='graph_filename', type='string', default=None)
opt_parser.add_option('-o','--mapfilename', dest='mapfilename', type='string', default=None)
opt_parser.add_option('--fc','--filtercloud', dest='filter_cloud', nargs=3 , type='float', default=[0, 0, 0])
opts, args = opt_parser.parse_args(sys.argv[1:])

# if opts.graph_filename is None:
	# raise Exception('no graph file is given')
	# exit()
if opts.mapfilename is None:
	raise Exception('no map file is given')
	exit()



# f_handle_open_map = open(os.path.join(opts.dir_prefix, opts.graph_filename), 'rb')
# data = f_handle_open_map.read()
# file = IcpTestTools.ZipInputStream(StringIO.StringIO(data))
# lines = file.read()
# graph = jsonpickle.decode(lines)

counter_index = opts.start_frame_no #1#21#30#x1
pcl_total_cloud = pcl.PointCloud().to_array()
while True:
	file_string = 'scan'+format(counter_index,'05d')+'.pcd'
	file_string = os.path.join(opts.dir_prefix, file_string)
	if not os.path.isfile(file_string):
		break
	pcl_cloud = pcl.load(file_string)
	# print pcl_cloud
	pcl_total_cloud = np.vstack((pcl_total_cloud , pcl_cloud.to_array()))
	# print pcl.PointCloud().from_array(pcl_total_cloud)
	# print pcl_total_cloud
	print counter_index
	
	counter_index = counter_index + 1
# print pcl_total_cloud
pcl_tosaved = pcl.PointCloud()
pcl_tosaved.from_array(pcl_total_cloud)
print pcl_tosaved
if opts.filter_cloud[0] != 0 and opts.filter_cloud[1] != 0 and opts.filter_cloud[2] !=0 :
	print 'doing voxel filtering'
	fil = pcl_tosaved.make_voxel_grid_filter()
	fil.set_leaf_size(opts.filter_cloud[0], opts.filter_cloud[1], opts.filter_cloud[2])
	pcl_tosaved = fil.filter()
	print pcl_tosaved

pcl.save(pcl_tosaved, os.path.join(opts.dir_prefix, opts.mapfilename+'.pcd'))
