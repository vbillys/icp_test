#! /usr/bin/env python
import IcpTestTools
import ujson
import jsonpickle

import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from foobar.msg import GraphLink, GraphStruct, GraphNode
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

def make_sure_path_exists(path):
	try:
		os.makedirs(path)
	except OSError as exception:
		if exception.errno != errno.EEXIST:
			raise

opt_parser = OptionParser()
opt_parser.add_option('-b','--bagfile', dest='bag_filename', type='string', default=None)
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('-f','--contfile', dest='graph_filename', type='string', default=None)
opt_parser.add_option('--nocloudsave', dest='nocloudsave',  action='store_true', default=False)
opts, args = opt_parser.parse_args(sys.argv[1:])

if opts.bag_filename is None: 
	raise Exception('no bag file is given')
	exit()
if opts.graph_filename is None:
	raise Exception('no graph file is given')
	exit()


def query_yes_no_quit(question, default="yes"):
    """Ask a yes/no/quit question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
	It must be "yes" (the default), "no", "quit" or None (meaning
			an answer is required of the user).

	The "answer" return value is one of "yes", "no" or "quit".
    """
    valid = {"yes":"yes",   "y":"yes",    "ye":"yes", "no":"no",     "n":"no", "quit":"quit", "qui":"quit", "qu":"quit", "q":"quit"}
    if default == None:
	prompt = " [y/n/q] "
    elif default == "yes":
	prompt = " [Y/n/q] "
    elif default == "no":
	prompt = " [y/N/q] "
    elif default == "quit":
	prompt = " [y/n/Q] "
    else:
	raise ValueError("invalid default answer: '%s'" % default)

    while 1:
	sys.stdout.write(question + prompt)
	choice = raw_input().lower()
	if default is not None and choice == '':
	    return default
	elif choice in valid.keys():
	    return valid[choice]
	else:
	    sys.stdout.write("Please respond with 'yes', 'no' or 'quit'.\n")

if len(opts.dir_prefix) == 0:
	print 'asking...'
	if query_yes_no_quit("no directory is specified, this will populate current one , continue? ", default="no") != "yes":
		exit()
else:
	if not os.path.exists(opts.dir_prefix):
		if query_yes_no_quit("thedirectory specified is not there, , create? ", default="no") != "yes":
			exit()
		else:
			make_sure_path_exists(opts.dir_prefix)


def euler_to_degrees(input):
  output = [0,0,0]
  output[0] = round(math.degrees(input[0]),3)
  output[1] = round(math.degrees(input[1]),3)
  output[2] = round(math.degrees(input[2]),3)
  return output

bag = rosbag.Bag(opts.bag_filename)
count_velo = 0
count_odom = 0
count_nodes = 0
last_pose = Pose()
prev_pose = Pose()
last_relpose = PoseWithCovariance()
last_cloud = list()
# graph = {}
graph_dict = {}
graph = GraphStruct()
first_flag = True
for topic, msg, t in bag.read_messages(topics=['velodyne_cloud_registered', 'aft_mapped_to_init']):
# for topic, msg, t in bag.read_messages(topics=['aft_mapped_to_init']):
	# print msg.header
	if topic=='velodyne_cloud_registered':
		count_velo = count_velo + 1
		last_cloud = pc2.read_points(msg, skip_nans=True)
		# print list(last_cloud)
		last_cloud = list(last_cloud)
		last_cloud_array = [[p[2],p[0],p[1]] for p in last_cloud]
		pcl_cloud = pcl.PointCloud()
		pcl_cloud.from_array(np.array(last_cloud_array, dtype=np.float32))
		# print pcl.cloud
	if topic=='aft_mapped_to_init':
		count_odom = count_odom + 1
		# print msg
		# quat = (msg.pose.pose.orientation.z, -msg.pose.pose.orientation.x, -msg.pose.pose.orientation.y, msg.pose.pose.orientation.w)
		quat = (msg.pose.pose.orientation.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.w)
		# euler = tf.transformations.euler_from_quaternion(quat)
		# hom_rot_mat_inv = tf.transformations.quaternion_matrix(quaternion_inverse(quat))
		hom_rot_mat= tf.transformations.quaternion_matrix(quat)
		euler = tf.transformations.euler_from_matrix(hom_rot_mat, 'sxyz')
		# euler[1] = -euler[1]
		# euler[2] = -euler[2]
		print euler_to_degrees(euler)
		# last_pose = copy.deepcopy(msg.pose.pose)
		prev_pose = last_pose
		last_pose = Pose()
		last_pose.orientation.x = quat[0]
		last_pose.orientation.y = quat[1]
		last_pose.orientation.z = quat[2]
		last_pose.orientation.w = quat[3]
		last_pose.position.x = msg.pose.pose.position.z
		last_pose.position.y = msg.pose.pose.position.x
		last_pose.position.z = msg.pose.pose.position.y
		hom_mat = hom_rot_mat
		hom_mat[0,3] = last_pose.position.x
		hom_mat[1,3] = last_pose.position.y
		hom_mat[2,3] = last_pose.position.z
		inv_hom_mat = np.linalg.inv(hom_mat)
	print t, topic, count_velo, count_odom
	if count_velo == count_odom and count_velo == count_nodes+1 and count_odom == count_nodes+1:
		count_nodes = count_nodes + 1
		print 'saving:',count_nodes
		# first built map, so surely increasing no of nodes, no loop close yet
		# if not graph: #Empty
		if not len(graph.nodes): #Empty
			this_new_node = GraphNode()
			this_new_node.node_name = str(count_nodes)
			this_new_node.pose=last_pose
			# graph[count_nodes] = [last_pose, []]
			# Ok we keep dict to fasten back reference later when building map
			# Ok to keep ordering (to refer back to rosified msg list), we add counter to the dict :)
			# graph_dict[count_nodes] = [last_pose, [], count_nodes-1]
			graph.nodes.append(this_new_node)
			if not opts.nocloudsave:
				pcl.save(pcl_cloud, os.path.join(opts.dir_prefix, 'scan'+format(count_nodes,'05d')+'.pcd'))
				cloud_T = np.ones((len(last_cloud_array),4))
				cloud_T[:,:-1] = pcl_cloud
				cloud_T=np.dot(cloud_T, inv_hom_mat.T)
				cloud_T=cloud_T[:,:-1]
				pcl_cloud_T = pcl.PointCloud()
				pcl_cloud_T.from_array(np.array(cloud_T, dtype=np.float32))
				pcl.save(pcl_cloud_T, os.path.join(opts.dir_prefix, 'scanorg'+format(count_nodes,'05d')+'.pcd'))
			if first_flag:
				first_flag = False
			else:
				# This should not happen
				raise Exception("graph still empty but this is not first initialization, quiting check!")
				exit()
		else:
			# if count_nodes-1 in graph_dict:
				# last_relpose = PoseWithCovariance()
				# graph_dict[count_nodes-1][1].append([count_nodes,last_relpose])
				# this_new_edge = GraphLink()
				# this_new_edge.connected_to = str(count_nodes)
				# graph.nodes[graph_dict[count_nodes-1][2]].edges.append(this_new_edge)
			# create for this new node
			# else:
				# # This should not happen
				# raise Exception("new node is about being added, but no previous reference!, aborting")
				# exit()
			# graph_dict[count_nodes] = [last_pose,[], count_nodes-1]
			this_new_node = GraphNode()
			this_new_node.node_name = str(count_nodes)
			this_new_node.pose=last_pose
			graph.nodes.append(this_new_node)
			if not opts.nocloudsave:
				pcl.save(pcl_cloud, os.path.join(opts.dir_prefix, 'scan'+format(count_nodes,'05d')+'.pcd'))
				cloud_T = np.ones((len(last_cloud_array),4))
				cloud_T[:,:-1] = pcl_cloud
				cloud_T=np.dot(cloud_T, inv_hom_mat.T)
				cloud_T=cloud_T[:,:-1]
				pcl_cloud_T = pcl.PointCloud()
				pcl_cloud_T.from_array(np.array(cloud_T, dtype=np.float32))
				pcl.save(pcl_cloud_T, os.path.join(opts.dir_prefix, 'scanorg'+format(count_nodes,'05d')+'.pcd'))

# print graph
# strlog = ujson.dumps(graph, ensure_ascii=False, double_precision=4)
# strlog = ujson.dumps(graph, ensure_ascii=False, double_precision=4, indent=2)
strlog = jsonpickle.encode(graph)
# print strlog

bag.close()

f_handle = open(os.path.join(opts.dir_prefix, opts.graph_filename),'w')
f_handle.write(zlib.compress(strlog))
f_handle.close()

f_handle_open_map = open(os.path.join(opts.dir_prefix, opts.graph_filename), 'rb')
data = f_handle_open_map.read()
file = IcpTestTools.ZipInputStream(StringIO.StringIO(data))
lines = file.read()
# print lines
graph = jsonpickle.decode(lines)
print graph
