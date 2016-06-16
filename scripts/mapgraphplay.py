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
opt_parser.add_option('-r','--rate', dest='publish_rate', type='float', default=1)
opt_parser.add_option('-s','--startframe', dest='start_frame_no', type='int', default=1)
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('-f','--contfile', dest='graph_filename', type='string', default=None)
opt_parser.add_option('--tc','--topiccloud', dest='cloud_topic', type='string', default='points')
opt_parser.add_option('--to','--topicodom', dest='odom_topic', type='string', default='/vmc_navserver/state')
opts, args = opt_parser.parse_args(sys.argv[1:])

if opts.graph_filename is None:
	raise Exception('no graph file is given')
	exit()

f_handle_open_map = open(os.path.join(opts.dir_prefix, opts.graph_filename), 'rb')
data = f_handle_open_map.read()
file = IcpTestTools.ZipInputStream(StringIO.StringIO(data))
lines = file.read()
# print lines
# graph = ujson.loads(lines)
graph = jsonpickle.decode(lines)
# print graph

rospy.init_node('map_frame_play', anonymous=False)
pub_frame   = rospy.Publisher(opts.cloud_topic, PointCloud2)
pub_odom = rospy.Publisher(opts.odom_topic, Odometry)
pub_tf = tf.TransformBroadcaster()

rate = rospy.Rate(opts.publish_rate)
counter_index = opts.start_frame_no #1#21#30#x1


while not rospy.is_shutdown():
	file_string = 'scan'+format(counter_index,'05d')+'.pcd'
	file_string = os.path.join(opts.dir_prefix, file_string)
	pcl_cloud = pcl.load(file_string)
	pcloud = PointCloud2()
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'velodyne' #'odom' #'pose' #'frame' #'velodyne'
	pcloud = pc2.create_cloud_xyz32(header, pcl_cloud.to_array())
	pose = graph[str(counter_index)][0]
	print pose
	odom = Odometry()
	odom.header = header
	odom.header.frame_id = 'map'
	odom.child_frame_id = 'velodyne'
	odom.pose.pose = pose
	pub_frame.publish(pcloud)
	pub_odom.publish(odom)
	pub_tf.sendTransform((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), rospy.Time.now(),'velodyne','map')
	counter_index = counter_index + 1
	rate.sleep()
