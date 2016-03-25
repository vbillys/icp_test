#!/usr/bin/env python
from cython_catkin_example import cython_catkin_example
import math
import numpy as np
import pylab as plt
import itertools
import curses
import pylab as plt
import matplotlib.animation as animation

from scipy.spatial import KDTree

from optparse import OptionParser
import os, sys


opt_parser = OptionParser()
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('-g','--gthresh', dest='g_thresh', type='float', default=10.)
opt_parser.add_option('-l','--lthresh', dest='g_thresh_local', type='float', default=1.)
opt_parser.add_option('-s','--start', dest='start_index',type='int')
opt_parser.add_option('-e','--end', dest='end_index', type='int')
opts, args = opt_parser.parse_args(sys.argv[1:])
start_index = 0
end_index = None
if opts.start_index is not None:
	start_index = int(opts.start_index)
if opts.end_index is not None:
	start_index = int(opts.end_index)
dir_prefix = opts.dir_prefix
print  opts, args

# exit()

# dir_prefix = '/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/'

# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-175207/icp_poses.txt','r')

f_handle = open(os.path.join(dir_prefix , 'icp_poses.txt'),'r')

# f_handle_w_pose = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_lm_poses.txt','w')
# f_handle_w_pose = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_lm_poses.txt','w')
# f_handle_w_pose = open('/home/avavav/avdata/alphard/medialink/20150918-175207/icp_lm_poses.txt','w')

f_handle_w_pose = open(os.path.join(dir_prefix , 'icp_lm_poses.txt'),'w')


def getFloatNumberFromReadLines(f_handle, no_params):
	f_content = f_handle.readlines()
	points = []
	for str_ in f_content:
		strs = str_.strip()
		point_ = map(float, strs.split())
		point = zip(*(iter(point_),) * no_params)
		points.append(point[0])
	# print points
	return points


def read2DPointsFromTextFile(filename):
	f_h = open(filename,'r')
	points = getFloatNumberFromReadLines(f_h, 2)
	return points


# offset = 1650
offset = start_index
points = getFloatNumberFromReadLines(f_handle, 12)
if end_index is None:
	end_index = len(points)
points = points[offset:end_index]

example = cython_catkin_example.PyCCExample()
def computeICPBetweenScans(no1,no2):
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no1)+'.txt')
	# test_cloud2 = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no2)+'.txt')
	test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_'+str(no1)+'.txt'))
	test_cloud2 = read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_'+str(no2)+'.txt'))
	cython_icp_result = example.processICP(np.array([x[0] for x in test_cloud], np.float32),np.array([x[1] for x in test_cloud], np.float32),np.array([x[0] for x in test_cloud2], np.float32),np.array([x[1] for x in test_cloud2], np.float32))
	return cython_icp_result

def transformCloud(test_cloud, tm):
	H_points = [[p[0], p[1],1] for p in test_cloud]
	M_points = np.matrix(H_points)
	point_t = tm *M_points.getT()
	point_t = point_t.getT().tolist()
	return point_t	

def accumulateIcpTransform(icp_result, x, y, yaw):
	x_icp =  math.cos(-icp_result[2])*icp_result[0] + math.sin(-icp_result[2])*icp_result[1] 
	y_icp = -math.sin(-icp_result[2])*icp_result[0] + math.cos(-icp_result[2])*icp_result[1]
	x_icp_g =  math.cos(-yaw)*x_icp + math.sin(-yaw)*y_icp + x
	y_icp_g = -math.sin(-yaw)*x_icp + math.cos(-yaw)*y_icp + y
	yaw_icp_g = yaw + icp_result[2]
	return x_icp_g, y_icp_g, yaw_icp_g

def createTransformMatrix(x,y,yaw):
	return np.matrix([[math.cos(yaw),math.sin(yaw),x],[-math.sin(yaw),math.cos(yaw),y],[0,0,1]])


# g_thresh = 5 #20.
# g_thresh_local = .5 #2.5 
g_thresh = opts.g_thresh #5 #20.
g_thresh_local = opts.g_thresh_local #.5 #2.5 
# last_x = 0
# last_y = 0
# last_yaw = 0
last_x = points[0][0]
last_y = points[0][1]
last_yaw = points[0][2]
next_capture_dist_local = 0- g_thresh_local
next_capture_dist=  g_thresh
# index_point = 0
index_point = offset
travelled_dist = 0 
accum_scan_indices = []
accum_scan_index = []

for point in points:
	# if index_point == 0:
		# yaw = 0
		# x = 0
		# y = 0
	# else:
	x = point[0]
	y = point[1]
	yaw=point[2]
	travelled_dist = travelled_dist + math.hypot(last_x - x, last_y - y)
	last_x = x
	last_y = y
	last_yaw = yaw
	if travelled_dist > next_capture_dist_local:
		while travelled_dist > next_capture_dist_local:
			next_capture_dist_local = next_capture_dist_local + g_thresh_local
		# print travelled_dist
		accum_scan_index.append (index_point)
	if travelled_dist > next_capture_dist:
		while travelled_dist > next_capture_dist:
			next_capture_dist = next_capture_dist + g_thresh
		# print "captured a local map", travelled_dist
		accum_scan_indices.append([accum_scan_index,index_point])
		accum_scan_index = []
	index_point = index_point + 1

total_indices =  len(accum_scan_indices)

def getScanPoints(no):
	# return read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	return read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_'+str(no)+'.txt'))

def getScanPointsFiltered(no):
	# return read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(no)+'.txt')
	# return read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_filtered_'+str(no)+'.txt')
	# return read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-175207/scan_filtered_'+str(no)+'.txt')
	return read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_filtered_'+str(no)+'.txt'))

def translateScan2D(points_2d, x, y):
	return [[xx[0] + x, xx[1] + y] for xx in points_2d]


def getLocalMap(indexes):
	ref_node_index = indexes[1]
	pose_local_map = list(points[ref_node_index-offset]) + [int(ref_node_index)]
	local_map = []
	for iid in indexes[0]:
		if iid == ref_node_index:
			local_map = local_map + getScanPointsFiltered(iid+1)
		else:
			untransformed = getScanPointsFiltered(iid+1)
			this_iid_pose = points[iid-offset]
			untransformed = transformCloud(untransformed, createTransformMatrix(this_iid_pose[0],this_iid_pose[1],-(this_iid_pose[2])))
			# untransformed = transformCloud(untransformed, createTransformMatrix(this_iid_pose[0],this_iid_pose[1],-(0)))
			# translated = translateScan2D(untransformed, this_iid_pose[0] - pose_local_map[0],this_iid_pose[1] - pose_local_map[1])
			translated = translateScan2D(untransformed,  - pose_local_map[0], - pose_local_map[1])
			# translated = translateScan2D(transformed , this_iid_pose[0] - pose_local_map[0],this_iid_pose[1] - pose_local_map[1])
			# transformed = transformCloud(translated, createTransformMatrix(0,0,-(this_iid_pose[2]- pose_local_map[2])))
			transformed = transformCloud(translated, createTransformMatrix(0,0,-(- pose_local_map[2])))
			local_map = local_map + transformed
			# local_map = local_map + translated
	return local_map, pose_local_map

# got the indices, now building local maps, interwoven with icp trees

def convertIntoCsvString(pose_local_map):
	return " ".join(format(x, ".4f") for x in pose_local_map) + "\n"

def saveCloud2DToFile(f_h, points_2d):
	f_h.write("".join([" ".join(format(x, ".6f") for x in p) + "\n" for p in points_2d]))

for ii in range(0,total_indices):
	local_map, pose_local_map = getLocalMap(accum_scan_indices[ii])
	f_handle_w_pose.write(convertIntoCsvString(pose_local_map))
	# f_lm = open('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(ii)+'.txt','w')
	# f_lm = open('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_lm_filtered_'+str(ii)+'.txt','w')
	# f_lm = open('/home/avavav/avdata/alphard/medialink/20150918-175207/scan_lm_filtered_'+str(ii)+'.txt','w')
	f_lm = open(os.path.join(dir_prefix , 'scan_lm_filtered_'+str(ii)+'.txt'),'w')
	saveCloud2DToFile(f_lm, local_map)


