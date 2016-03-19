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


## $ python zlib-example-4.py
## We will perhaps eventually be writing only small
## modules which are identified by name as they are
## used to build larger ones, so that devices like
## indentation, rather than delimiters, might become
## feasible for expressing local structure in the
## source language.
##     -- Donald E. Knuth, December 1974

f_handle_pose = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.txt','r')
f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.graph','r')
f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-initial.graph','r')
f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-final.graph','r')
# f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm.graph','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm-treeopt-initial.graph','r')
# f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm-treeopt-final.graph','r')


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

# ng = 43 # 2433
ng = 1700 #  2433

vertices2 =  getVertexFromGraph(f_handle2, ng)
vertices =  getVertexFromGraph(f_handle, ng, True)
vertices3 =  getVertexFromGraph(f_handle3, ng, True)
points = getFloatNumberFromReadLines(f_handle_pose, 12)

fig, ax = plt.subplots()
ax.axis([-200, 100, -100, 200])
ax.set_aspect('equal','datalim')
# ax.hold(True)
# plt.ion()
# fig.canvas.draw()
# plt.show(False)
point_t = []
checkpoints = 0
# plt.plot([o[1] for o in vertices],[o[2] for o in vertices])
# plt.plot([o[1] for o in vertices2],[o[2] for o in vertices2])
plt.plot([o[1] for o in vertices3],[o[2] for o in vertices3])
# # plt.plot([o[0] for o in points],[o[1] for o in points])

g_thresh = 10
travelled_dist = 0 
last_x = 0
last_y = 0
next_capture_dist = 0- g_thresh
for vertex in vertices3:
# for point in points:
	if checkpoints % 1 == 0: #25
		# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(vertex[0])+'.txt')
		test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(vertex[0])+'.txt')
		# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(checkpoints)+'.txt')
		travelled_dist = travelled_dist + math.hypot(last_x - vertex[1], last_y - vertex[2])
		last_x = vertex[1]
		last_y = vertex[2]
		if travelled_dist > next_capture_dist:
			while travelled_dist > next_capture_dist:
				next_capture_dist = next_capture_dist + g_thresh
			t_point_t = transformCloud(test_cloud, createTransfromFromXYYaw(vertex[1],vertex[2],-vertex[3]))
			# t_point_t = transformCloud(test_cloud, createTransfromFromXYYaw(point[0],point[1],-point[2]))
			point_t = point_t + t_point_t
	checkpoints = checkpoints + 1
ax.scatter ([x[0] for x in point_t],[x[1] for x in point_t], color='green', s=.3)
# fig.canvas.draw()
plt.show(True)

def saveCloud2DToFile(f_h, points_2d):
	f_h.write("".join([" ".join(format(x, ".6f") for x in p) + "\n" for p in points_2d]))

def saveCloud2DToFileCompressed(f_h, points_2d):
	f_h.write(zlib.compress("".join([" ".join(format(x, ".6f") for x in p) + "\n" for p in points_2d]), 5))

# f_handle_save_map = open('/home/avavav/avdata/alphard/medialink/20150918-180619/map.txt', 'w')
# # saveCloud2DToFile(f_handle_save_map, point_t)
# saveCloud2DToFileCompressed(f_handle_save_map, point_t)


start = time.time()
f_handle_open_map = open('/home/avavav/avdata/alphard/medialink/20150918-180619/map.txt', 'rb')
data = f_handle_open_map.read()
file = ZipInputStream(StringIO.StringIO(data))
lines = file.readlines()
end = time.time()
print end-start
# for line in lines:
	# print line[:-1]
