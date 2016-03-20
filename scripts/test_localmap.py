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

f_handle_processed_graph = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-final.graph','r')
f_handle_open_map = open('/home/avavav/avdata/alphard/medialink/20150918-180619/map.txt', 'rb')
data_map = f_handle_open_map.read()
file_openmap = ZipInputStream(StringIO.StringIO(data_map))
lines_map = file_openmap.readlines()
point_t = read2DPointsFromStringReadLines(lines_map)

fig, ax = plt.subplots()
ax.axis([-200, 100, -100, 200])
ax.set_aspect('equal','datalim')
ax.scatter ([x[0] for x in point_t],[x[1] for x in point_t], color='green', s=.3)

graphVertices = getVertexFromGraph(f_handle_processed_graph, 1700, True)

test_poses = getFloatNumberFromReadLines(open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_lm_poses.txt','r'), 13)
# n = 1 
vertices_plot = []
odom_plot = []
only_odom_accum_plot = []
icp_plot = []
# initial_error = [5,5,math.radians(10)]
for n in range (10, len(test_poses)):
	test_local_map = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_' + str(n) + '.txt')
	# print test_poses[n]
	random_x = 0#np.random.normal(0,.5)
	random_y = 0#np.random.normal(0,.5)
	random_yaw = 0#np.random.normal(0,math.radians(5))

	# icp_pose = computeICPBetweenScans(point_t, test_local_map, test_poses[n][0], test_poses[n][1], test_poses[n][2])
	# icp_pose = computeICPBetweenScans(point_t, test_local_map)
	# print icp_pose
	# print graphVertices[int(test_poses[n][-1])]

	only_odom_accum_plot.append([test_poses[n][0]+random_x,test_poses[n][1]+random_y])

	if n == 10:
		random_x = np.random.normal(0,10.5)
		random_y = np.random.normal(0,10.5)
		random_yaw = np.random.normal(0,math.radians(35))
		# random_x = initial_error[0]
		# random_y = initial_error[1]
		# random_yaw = initial_error[2]
		odom_plot.append([test_poses[n][0]+random_x,test_poses[n][1]+random_y])
		# odom_plot.append([test_poses[n][0],test_poses[n][1]])
		icp_pose = computeICPBetweenScans(point_t, test_local_map, test_poses[n][0]+random_x, test_poses[n][1]+random_y, test_poses[n][2]+random_yaw)
	else:
		_x = test_poses[n][0] - test_poses[n-1][0] + icp_pose[0]+random_x
		_y = test_poses[n][1] - test_poses[n-1][1] + icp_pose[1]+random_y
		_yaw = test_poses[n][2] - test_poses[n-1][2] + icp_pose[2]+random_yaw
		odom_plot.append([_x,_y])
		icp_pose = computeICPBetweenScans(point_t, test_local_map, _x, _y, _yaw)
		# icp_pose = computeICPBetweenScans(point_t, test_local_map, test_poses[n][0]+random_x, test_poses[n][1]+random_y, test_poses[n][2]+random_yaw)

	vertices_plot.append([graphVertices[int(test_poses[n][-1])][1],graphVertices[int(test_poses[n][-1])][2]])
	icp_plot.append([icp_pose[0],icp_pose[1]])

# test_local_map_aligned = transformCloud(test_local_map, createTransfromFromXYYaw(icp_pose[0], icp_pose[1], -icp_pose[2]))
# ax.scatter ([x[0] for x in test_local_map_aligned ],[x[1] for x in test_local_map_aligned ], color='blue', s=1.3)

plt.plot([x[0] for x in only_odom_accum_plot], [x[1] for x in only_odom_accum_plot], color='orange', lw=3, linestyle='--')
plt.plot([x[0] for x in odom_plot], [x[1] for x in odom_plot], color='blue', lw=3)
plt.plot([x[0] for x in vertices_plot], [x[1] for x in vertices_plot], color='red', lw=3, marker = '^', ms=10)
plt.plot([x[0] for x in icp_plot], [x[1] for x in icp_plot], color='cyan', linestyle='-', marker='o', lw=2)

plt.show(True)

