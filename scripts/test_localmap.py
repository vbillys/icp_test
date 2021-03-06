#!/usr/bin/env python


# import matplotlib
# matplotlib.use('GTKAgg')

import math
import numpy as np
import pylab as plt
import itertools
import curses
import pylab as plt
import matplotlib.animation as animation
import time
import zlib

import string, StringIO

from cython_catkin_example import cython_catkin_example


from optparse import OptionParser
import os, sys

import IcpTestTools

opt_parser = OptionParser()
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('-m','--mapdirectory', dest='dir_prefix_map', type='string', default='')
opt_parser.add_option('-c','--axeslimit', dest='plot_axes_limit', nargs=4 , type='float')
opt_parser.add_option('-i','--initial_pose', dest='initial_pose', nargs=3 , type='float')
opt_parser.add_option('--nm', dest='no_matching',  action='store_true', default=False)
opt_parser.add_option('--sa', dest='save_add_to_graph',  action='store_true', default=False)
opts, args = opt_parser.parse_args(sys.argv[1:])
dir_prefix = opts.dir_prefix
dir_prefix_map = opts.dir_prefix_map
print  opts, args

class ZipInputStream:

	def __init__(self, file):
		self.file = file
		self.__rewind()

	def __rewind(self):
		self.zip = zlib.decompressobj()
		self.pos = 0 # position in zipped stream
		self.offset = 0 # position in unzipped stream
		self.data = ""

	def __fill(self, bytes):
		if self.zip:
			# read until we have enough bytes in the buffer
			while not bytes or len(self.data) < bytes:
				self.file.seek(self.pos)
				data = self.file.read(16384)
				if not data:
					self.data = self.data + self.zip.flush()
					self.zip = None # no more data
					break
				self.pos = self.pos + len(data)
				self.data = self.data + self.zip.decompress(data)

	def seek(self, offset, whence=0):
		if whence == 0:
			position = offset
		elif whence == 1:
			position = self.offset + offset
		else:
			raise IOError, "Illegal argument"
		if position < self.offset:
			raise IOError, "Cannot seek backwards"

		# skip forward, in 16k blocks
		while position > self.offset:
			if not self.read(min(position - self.offset, 16384)):
				break

	def tell(self):
		return self.offset

	def read(self, bytes = 0):
		self.__fill(bytes)
		if bytes:
			data = self.data[:bytes]
			self.data = self.data[bytes:]
		else:
			data = self.data
			self.data = ""
		self.offset = self.offset + len(data)
		return data

	def readline(self):
		# make sure we have an entire line
		while self.zip and "\n" not in self.data:
			self.__fill(len(self.data) + 512)
		i = string.find(self.data, "\n") + 1
		if i <= 0:
			return self.read()
		return self.read(i)

	def readlines(self):
		lines = []
		while 1:
			s = self.readline()
			if not s:
				break
			lines.append(s)
		return lines


def getFloatNumberFromStringReadLines(strs, no_params):
	points = []
	for str_ in strs:
		strs = str_.strip()
		point_ = map(float, strs.split())
		point = zip(*(iter(point_),) * no_params)
		points.append(point[0])
	# print points
	return points

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

def read2DPointsFromStringReadLines(strs):
	points = getFloatNumberFromStringReadLines(strs, 2)
	return points

def read2DPointsFromTextFile(filename):
	f_h = open(filename,'r')
	points = getFloatNumberFromReadLines(f_h, 2)
	return points

def computeICPBetweenScans(test_cloud,test_cloud2, init_x = 0 , init_y = 0, init_yaw = 0):
	example = cython_catkin_example.PyCCExample()
	cython_icp_result = example.processICP(np.array([x[0] for x in test_cloud], np.float32),np.array([x[1] for x in test_cloud], np.float32),np.array([x[0] for x in test_cloud2], np.float32),np.array([x[1] for x in test_cloud2], np.float32), init_x, init_y, init_yaw)
	return cython_icp_result

def transformCloud(test_cloud, tm):
	H_points = [[p[0], p[1],1] for p in test_cloud]
	M_points = np.matrix(H_points)
	point_t = tm *M_points.getT()
	point_t = point_t.getT().tolist()
	point_t = [[p[0], p[1]] for p in point_t]
	return point_t	

def createTransfromFromXYYaw(x,y,yaw):
	return np.matrix([[math.cos(yaw),math.sin(yaw),x],[-math.sin(yaw),math.cos(yaw),y],[0,0,1]])


def getVertexFromGraph(f_handle, no_vertex, processed=False):
	f_content = f_handle.readlines()
	points = []
	# for str_ in f_content:
	for i in range(0,no_vertex):
		strs = f_content[i].strip()
		strs_splitted =  strs.split()
		noid = int(strs_splitted[1])
		if processed:
			coord = [-float(strs_splitted[2]), float(strs_splitted[3]), -float(strs_splitted[4])]
		else:
			coord = [float(strs_splitted[2]), float(strs_splitted[3]), float(strs_splitted[4])]
		points.append([noid]+coord)
	return points

# dir_prefix = '/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/'

# f_handle_processed_graph = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-final.graph','r')
# f_handle_open_map = open('/home/avavav/avdata/alphard/medialink/20150918-180619/map.txt', 'rb')

f_handle_open_map = open(os.path.join(dir_prefix_map , 'map.txt'), 'rb')

data_map = f_handle_open_map.read()
file_openmap = ZipInputStream(StringIO.StringIO(data_map))
lines_map = file_openmap.readlines()
point_t = read2DPointsFromStringReadLines(lines_map)

fig, ax = plt.subplots()
if opts.plot_axes_limit is not None:
	ax.axis(opts.plot_axes_limit)
# ax.axis([-200, 100, -100, 200])
ax.set_aspect('equal','datalim')
ax.scatter ([x[0] for x in point_t],[x[1] for x in point_t], color='green', s=.3)

# graphVertices = getVertexFromGraph(f_handle_processed_graph, 1700, True)



# test_poses = getFloatNumberFromReadLines(open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_lm_poses.txt','r'), 13)
# test_poses = getFloatNumberFromReadLines(open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_lm_poses.txt','r'), 13)
# test_poses = getFloatNumberFromReadLines(open('/home/avavav/avdata/alphard/medialink/20150918-175207/icp_lm_poses.txt','r'), 13)

# test_poses = getFloatNumberFromReadLines(open(os.path.join(dir_prefix , 'icp_lm_poses.txt'),'r'), 13)
test_poses = IcpTestTools.getFloatNumberListFromReadLines(open(os.path.join(dir_prefix , 'icp_lm_poses.txt'),'r'))
# n = 1 
vertices_plot = []
odom_plot = []
only_odom_accum_plot = []
icp_plot = []
# initial_error = [5,5,math.radians(10)]
# initial_error = [-30,0,0] #-15
initial_error = [0,0,0] #-15
if opts.initial_pose is not None:
	init_pose_x = opts.initial_pose[0]
	init_pose_y = opts.initial_pose[1]
	init_pose_yaw = math.radians(opts.initial_pose[2])
else:
	init_pose_x = 0
	init_pose_y = 0
	init_pose_yaw = 0

def translateScan2D(points_2d, x, y):
	return [[xx[0] + x, xx[1] + y] for xx in points_2d]

def projectToInitPose(pose,pose0, cloud, init_x, init_y, init_yaw):
	#rotate first
	# _x = pose[0]-pose0[0]
	# _y = pose[1]-pose0[1]
	# _yaw = pose[2] - pose0[2]
	_cloud = transformCloud(cloud, createTransfromFromXYYaw(pose[0], pose[1], -pose[2]))
	# _cloud = translateScan2D(_cloud, -pose0[0]+init_x, -pose0[1]+init_y)
	_cloud = translateScan2D(_cloud, -pose0[0], -pose0[1])
	# _cloud = transformCloud(_cloud, createTransfromFromXYYaw(0, 0, _yaw))
	_cloud = transformCloud(_cloud, createTransfromFromXYYaw(0, 0, -init_yaw))
	_cloud = translateScan2D(_cloud, init_x, init_y)
	# _x = -pose[0]+init_x
	# _y = -pose[1]+init_y
	# return transformCloud(_cloud, createTransfromFromXYYaw(_x, _y, 0))
	return _cloud

# print test_poses
if opts.initial_pose is not None:
	test_poses_inited = [ list(jj) for jj in test_poses]
	idx = 0
	for p in test_poses_inited:
		_x = p[0]-test_poses[0][0]
		_y = p[1]-test_poses[0][1]
		_yaw = p[2] #- test_poses[0][2]
		_xx = math.cos(init_pose_yaw)*_x - math.sin(init_pose_yaw)*_y
		_yy = math.sin(init_pose_yaw)*_x + math.cos(init_pose_yaw)*_y
		# test_poses[idx] = list(test_poses[idx])
		test_poses_inited[idx][0] = _xx + init_pose_x
		test_poses_inited[idx][1] = _yy + init_pose_y
		test_poses_inited[idx][2] = _yaw + init_pose_yaw
		idx = idx + 1
	pose0 = test_poses[0]



for n in range (0, len(test_poses)):
	# test_local_map = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_' + str(n) + '.txt')
	# test_local_map = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_lm_filtered_' + str(n) + '.txt')
	# test_local_map = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-175207/scan_lm_filtered_' + str(n) + '.txt')

	test_local_map = read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_lm_filtered_' + str(n) + '.txt'))

	# print test_poses[n]
	random_x = 0#np.random.normal(0,.5)
	random_y = 0#np.random.normal(0,.5)
	random_yaw = 0#np.random.normal(0,math.radians(5))

	# icp_pose = computeICPBetweenScans(point_t, test_local_map, test_poses[n][0], test_poses[n][1], test_poses[n][2])
	# icp_pose = computeICPBetweenScans(point_t, test_local_map)
	# print icp_pose
	# print graphVertices[int(test_poses[n][-1])]

	if opts.initial_pose is not None:
		# only_odom_accum_plot.append([test_poses[n][0]-pose0[0]+random_x+init_pose_x,test_poses[n][1]-pose0[1]+random_y+init_pose_y])
		only_odom_accum_plot.append([test_poses_inited[n][0]+random_x,test_poses_inited[n][1]+random_y])
	else:
		only_odom_accum_plot.append([test_poses[n][0]+random_x,test_poses[n][1]+random_y])

	if n == 0:
		# random_x = np.random.normal(0,10.5)
		# random_y = np.random.normal(0,10.5)
		# random_yaw = np.random.normal(0,math.radians(35))
		random_x = initial_error[0]
		random_y = initial_error[1]
		random_yaw = initial_error[2]
		if not opts.no_matching:
			if opts.initial_pose is not None:
				odom_plot.append([random_x+init_pose_x,random_y+init_pose_y])
			else:
				odom_plot.append([test_poses[n][0]+random_x,test_poses[n][1]+random_y])
			# odom_plot.append([test_poses[n][0],test_poses[n][1]])
			if opts.initial_pose is not None:
				# icp_pose = computeICPBetweenScans(point_t, test_local_map, init_pose_x, init_pose_y, init_pose_yaw)
				icp_pose = computeICPBetweenScans(point_t, test_local_map, init_pose_x, init_pose_y, init_pose_yaw+test_poses[0][2])
			else:
				icp_pose = computeICPBetweenScans(point_t, test_local_map, test_poses[n][0]+random_x, test_poses[n][1]+random_y, test_poses[n][2]+random_yaw)
	else:
		if not opts.no_matching:
			if opts.initial_pose is not None:
				_x = test_poses_inited[n][0] - test_poses_inited[n-1][0] + icp_pose[0]+random_x
				_y = test_poses_inited[n][1] - test_poses_inited[n-1][1] + icp_pose[1]+random_y
				_yaw = test_poses_inited[n][2] - test_poses_inited[n-1][2] + icp_pose[2]+random_yaw
				odom_plot.append([_x,_y])
				icp_pose = computeICPBetweenScans(point_t, test_local_map, _x, _y, _yaw)
			else:
				_x = test_poses[n][0] - test_poses[n-1][0] + icp_pose[0]+random_x
				_y = test_poses[n][1] - test_poses[n-1][1] + icp_pose[1]+random_y
				_yaw = test_poses[n][2] - test_poses[n-1][2] + icp_pose[2]+random_yaw
				odom_plot.append([_x,_y])
				icp_pose = computeICPBetweenScans(point_t, test_local_map, _x, _y, _yaw)
			# icp_pose = computeICPBetweenScans(point_t, test_local_map, test_poses[n][0]+random_x, test_poses[n][1]+random_y, test_poses[n][2]+random_yaw)

	if opts.no_matching:
		if opts.initial_pose is None:
			t_point_t = transformCloud(test_local_map, createTransfromFromXYYaw(test_poses[n][0],test_poses[n][1],-test_poses[n][2]))
		else:
			# t_point_t = test_local_map_inited
			test_local_map_inited = projectToInitPose(test_poses[n], pose0,test_local_map, init_pose_x, init_pose_y, init_pose_yaw)
			# t_point_t = transformCloud(test_local_map, createTransfromFromXYYaw(test_poses[n][0]-pose0[0]+init_pose_x,test_poses[n][1]-pose0[1]+init_pose_y,-test_poses[n][2]+ pose0[2] + init_pose_yaw))
			t_point_t = test_local_map_inited
	else:
		t_point_t = transformCloud(test_local_map, createTransfromFromXYYaw(icp_pose[0],icp_pose[1],-icp_pose[2]))

	ax.scatter ([x[0] for x in t_point_t],[x[1] for x in t_point_t], color='magenta', s=.25)

	# vertices_plot.append([graphVertices[int(test_poses[n][-1])][1],graphVertices[int(test_poses[n][-1])][2]])
	if not opts.no_matching:
		icp_plot.append([icp_pose[0],icp_pose[1], icp_pose[2]])

# test_local_map_aligned = transformCloud(test_local_map, createTransfromFromXYYaw(icp_pose[0], icp_pose[1], -icp_pose[2]))
# ax.scatter ([x[0] for x in test_local_map_aligned ],[x[1] for x in test_local_map_aligned ], color='blue', s=1.3)

plt.plot([x[0] for x in only_odom_accum_plot], [x[1] for x in only_odom_accum_plot], color='orange', lw=3, linestyle='--')
plt.plot([x[0] for x in odom_plot], [x[1] for x in odom_plot], color='blue', lw=3)
# plt.plot([x[0] for x in vertices_plot], [x[1] for x in vertices_plot], color='red', lw=3, marker = '^', ms=10)
plt.plot([x[0] for x in icp_plot], [x[1] for x in icp_plot], color='cyan', linestyle='-', marker='o', lw=2)

print only_odom_accum_plot
print odom_plot
# print icp_pose

plt.show(True)

if not opts.save_add_to_graph:
	exit()


map_obj = IcpTestTools.MapScan(dir_prefix_map)
test_lm_obj = IcpTestTools.MapScan(dir_prefix, use_accumulated=True)
for n in range (0, len(test_poses)):
	# str_vertex_new = str_vertex_new + IcpTestTools.createToroVertexString(n, icp_plot[n][0], icp_plot[n][1], icp_plot[n][2])
	test_lm_obj.modPose(n, icp_plot[n], with_original = True)

# map_obj.loadGraphVertices(use_processed_map = True, reverse_processed_map = True)
# map_obj.loadGraphVertices(use_processed_map = False, reverse_processed_map = True, remove_zero_vertex = False)


# map_obj.loadGraphEdges(use_processed_map = True)

# print map_obj.loadGraphVertices()
# print test_lm_obj.getAllOriginalPoses()
# print map_obj.appendVerticesWithSetPoses(test_lm_obj.getAllOriginalPoses(), use_original_index = True)

# print map_obj.edges

map_obj.appendVerticesWithSetPoses(test_lm_obj.getAllOriginalPoses(), use_original_index = True)
# map_obj.appendVerticesWithSetPoses(test_lm_obj.getAllOriginalPoses(), start_id = 1651)
map_obj.appendEdgesWithVertices(overwrite_duplicate = False)

map_obj.loadGraphEdges(use_processed_map = True, add_to_current = True, reverse_processed_map = True)
# map_obj.loadGraphEdges(use_processed_map = True, add_to_current = True, override_covariances= True)
# map_obj.loadGraphEdges(use_processed_map = True, add_to_current = True)

# print map_obj.vertices, len(map_obj.vertices)
# print map_obj.appendEdgesWithVertices(), len(map_obj.edges)
# map_obj.loadGraphVertices(use_processed_map = True, reverse_processed_map = True)
# map_obj.loadGraphVertices(use_processed_map = True, reverse_processed_map = True, add_to_current=True)
map_obj.loadGraphVertices(use_processed_map = True, reverse_processed_map = True, add_to_current=True, remove_zero_vertex = False)

# map_obj.manuallyAddEdge(1650, 1651)
# map_obj.manuallyAddEdge(1650, 2230)
# map_obj.manuallyAddEdge(1250, 2230)
map_obj.manuallyAddEdge(0, 2230, override_information = 1000000000)
map_obj.manuallyAddEdge(0, 2445, override_information = 1000000000)

print len(map_obj.vertices), len(map_obj.edges)
# print map_obj.vertices[655:657]
# print map_obj.edges[(655,656)]


map_obj.createVertexStringFromVertices(include_zero_vertex = False)
# map_obj.createVertexStringFromVertices()
map_obj.createEdgeStringFromEdges()
map_obj.saveGraphVertices()
map_obj.saveGraphEdges()

# str_vertex_new = test_lm_obj.createVertexStringFromPoses(include_zero_vertex = False, start_index = 1, use_original = True, use_original_index = True)
# str_vertex_new = test_lm_obj.createVertexStringFromPoses(include_zero_vertex = False, start_index = 5000, use_original = True)
# print str_vertex_new
# print test_lm_obj.createVertexStringFromPoses(include_zero_vertex = False)
