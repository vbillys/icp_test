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

import sys, getopt
from optparse import OptionParser
import os

# try:
	# print 'input directory set to ' + sys.argv[1]
	# dir_prefix = sys.argv[1]
# except:
	# print "input directory needed, no input on this, forcing current!"
	# dir_prefix = ''

# try:
	# opts, args = getopt.getopt(sys.argv[2:],"s:e:")
# except:
	# pass
	
	# print "for some reason cannot get options, fallback to default values"




opt_parser = OptionParser()
# opt_parser.add_option('-c','--axeslimit', dest='plot_axes_limit', nargs=4 , type='float', default=[-200, 100, -100, 200])
opt_parser.add_option('-c','--axeslimit', dest='plot_axes_limit', nargs=4 , type='float')
# opt_parser.add_option('-c','--axeslimit', dest='plot_axes_limit_set',  action='store_true', default=False)
opt_parser.add_option('--sm', dest='save_map',  action='store_true', default=False)
opt_parser.add_option('--pm', dest='use_processed_map',  action='store_true', default=False)
opt_parser.add_option('--ua', dest='use_accumulated',  action='store_true', default=False)
opt_parser.add_option('--rpm', dest='reverse_processed_map',  action='store_true', default=False)
opt_parser.add_option('-s','--start', dest='start_index',type='int')
opt_parser.add_option('-e','--end', dest='end_index', type='int')
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('--ogifn', dest='initial_graph_filename', type='string', default=None)
opt_parser.add_option('--ogffn', dest='final_graph_filename', type='string', default=None)
opt_parser.add_option('-g','--gthresh', dest='g_thresh', type='float', default=10.)
opts, args = opt_parser.parse_args(sys.argv[1:])

dir_prefix = opts.dir_prefix
start_index = 0
end_index = None
print sys.argv[2:], opts, args

# for opt, arg in opts:
	# if opt == '-s':
		# start_index = int(arg)
	# elif opt == '-e':
		# end_index = int(arg)
if opts.start_index is not None:
	start_index = int(opts.start_index)
if opts.end_index is not None:
	end_index = int(opts.end_index)

print start_index, end_index
# exit()

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

# dir_prefix = '/home/avavav/avdata/alphard/onenorth/onenorth_wb_2016-03-24-10-05-56/'


# f_handle_pose = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.txt','r')
# f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.graph','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-initial.graph','r')
# f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-final.graph','r')

# f_handle_pose = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_poses.txt','r')
# f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_poses.graph','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_poses-treeopt-initial.graph','r')
# f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_poses-treeopt-final.graph','r')

# f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-175207/icp_poses.graph','r')

# f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm.graph','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm-treeopt-initial.graph','r')
# f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm-treeopt-final.graph','r')

# f_handle2 = open('/home/avavav/avdata/mobile_base/skygarden1/icp_poses.graph','r')

# f_handle2 = open('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/icp_poses.graph','r')
# f_handle2 = open('/home/avavav/avdata/alphard/onenorth/20150821-120036_sss/icp_poses.graph','r')
# f_handle2 = open('/home/avavav/avdata/alphard/onenorth/20150821-115715_sss/icp_poses.graph','r')
# f_handle2 = open('/home/avavav/avdata/alphard/onenorth/20150821-115401_sss/icp_poses.graph','r')
# f_handle2 = open('/home/avavav/avdata/alphard/onenorth/20150821-115223_sss/icp_poses.graph','r')

if opts.initial_graph_filename is not None:
	f_handle2 = open(os.path.join(dir_prefix, opts.initial_graph_filename),'r')
else:
	if opts.use_accumulated:
		f_handle2 = open(os.path.join(dir_prefix , 'icp_lm_poses.graph'),'r')
	else:
		f_handle2 = open(os.path.join(dir_prefix , 'icp_poses.graph'),'r')

if opts.final_graph_filename is not None:
	f_handle3 = open(os.path.join(dir_prefix, opts.final_graph_filename),'r')
else:
	
	if opts.use_processed_map:
		# f_handle3 = open('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/icp_poses-treeopt-initial.graph','r')
		if opts.use_accumulated:
			f_handle3 = open(os.path.join(dir_prefix, 'icp_lm_poses-treeopt-final.graph'),'r')
		else:
			f_handle3 = open(os.path.join(dir_prefix, 'icp_poses-treeopt-final.graph'),'r')

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

def getVertexFromGraphAutoCount(f_handle,  processed=False):
	f_content = f_handle.readlines()
	points = []
	# for str_ in f_content:
	for i in range(0,len(f_content)):
		strs = f_content[i].strip()
		strs_splitted =  strs.split()
		# print strs_splitted
		if strs_splitted[0] == "VERTEX" or strs_splitted[0] == "VERTEX2":
			noid = int(strs_splitted[1])
			if processed:
				coord = [-float(strs_splitted[2]), float(strs_splitted[3]), -float(strs_splitted[4])]
			else:
				coord = [float(strs_splitted[2]), float(strs_splitted[3]), float(strs_splitted[4])]
			points.append([noid]+coord)
	return points

ng = 1651

# ng = 43 # 2433
# ng = 1650#2677 #1650 # 2677 #1650 #2677 #152 #1999 #1015 #1700 #  2433
# ng = 1155#2368 #2451 #3000 #3942
# ng = 900

# ng = 2677


vertices2 =  getVertexFromGraphAutoCount(f_handle2)
if opts.use_processed_map:
	vertices3 =  getVertexFromGraphAutoCount(f_handle3, opts.reverse_processed_map)
	# vertices3 =  getVertexFromGraphAutoCount(f_handle3)

# vertices2 =  getVertexFromGraph(f_handle2, ng)
# vertices =  getVertexFromGraph(f_handle, ng, True)
# vertices3 =  getVertexFromGraph(f_handle3, ng, True)
# points = getFloatNumberFromReadLines(f_handle_pose, 12)

if end_index is None:
	end_index = len(vertices2)
vertices2 = vertices2[start_index: end_index]
if opts.use_processed_map:
	vertices3 = vertices3[start_index: end_index]

fig, ax = plt.subplots()
# ax.axis([-200, 100, -100, 200])
if opts.plot_axes_limit is not None:
	ax.axis(opts.plot_axes_limit)
# ax.axis([-80, 250, -350, 25])
ax.set_aspect('equal','datalim')
# ax.hold(True)
# plt.ion()
# fig.canvas.draw()
# plt.show(False)
point_t = []
checkpoints = 0
# plt.plot([o[1] for o in vertices],[o[2] for o in vertices])
plt.plot([o[1] for o in vertices2],[o[2] for o in vertices2])
if opts.use_processed_map:
	plt.plot([o[1] for o in vertices3],[o[2] for o in vertices3])
# # plt.plot([o[0] for o in points],[o[1] for o in points])

# lm_coords = getFloatNumberFromReadLines(open(dir_prefix+'icp_lm_poses.txt','r'),13)

g_thresh = opts.g_thresh #1 #10. #.5 #10 
# g_thresh = 10 #1 #10. #.5 #10 
travelled_dist = 0 
# last_x = 0
# last_y = 0
# last_x = vertices2[0][0]
# last_y = vertices2[0][1]
next_capture_dist = 0- g_thresh
if opts.use_processed_map:
	vertices_chosen = vertices3
	last_x = vertices3[0][0]
	last_y = vertices3[0][1]
else:
	vertices_chosen = vertices2
	last_x = vertices2[0][0]
	last_y = vertices2[0][1]
if opts.use_accumulated:
	for vertex in vertices_chosen[1:]:
		print vertex
		test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_lm_filtered_'+str(vertex[0]-1)+'.txt'))
		t_point_t = transformCloud(test_cloud, createTransfromFromXYYaw(vertex[1],vertex[2],-vertex[3]))
		point_t = point_t + t_point_t
else:
	for vertex in vertices_chosen:
	# for point in points:
		if checkpoints % 1 == 0: #25
		# if not (checkpoints == 0 and opts.use_accumulated):
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-175207/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(checkpoints)+'.txt')

			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/mobile_base/skygarden1/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/mobile_base/skygarden1/scan_'+str(vertex[0])+'.txt')

			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-120036_sss/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-115715_sss/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-115401_sss/scan_filtered_'+str(vertex[0])+'.txt')
			# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-115223_sss/scan_filtered_'+str(vertex[0])+'.txt')


			# if opts.use_accumulated:
				# test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_lm_filtered_'+str(vertex[0])+'.txt'))
			# else:
			test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix , 'scan_filtered_'+str(vertex[0])+'.txt'))

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

if not opts.save_map:
	exit()

# f_handle_save_map = open('/home/avavav/avdata/alphard/medialink/20150918-180619/map.txt', 'w')
# f_handle_save_map = open('/home/avavav/avdata/alphard/medialink/20150918-174721/map.txt', 'w')
# f_handle_save_map = open('/home/avavav/avdata/alphard/medialink/20150918-175207/map.txt', 'w')
# f_handle_save_map = open('/home/avavav/avdata/mobile_base/skygarden1/map.txt', 'w')

# f_handle_save_map = open('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/map.txt', 'w')

f_handle_save_map = open(os.path.join(dir_prefix, 'map.txt'), 'w')

# saveCloud2DToFile(f_handle_save_map, point_t)
saveCloud2DToFileCompressed(f_handle_save_map, point_t)


start = time.time()
# f_handle_open_map = open('/home/avavav/avdata/alphard/medialink/20150918-180619/map.txt', 'rb')
# f_handle_open_map = open('/home/avavav/avdata/alphard/medialink/20150918-174721/map.txt', 'rb')
# f_handle_open_map = open('/home/avavav/avdata/alphard/medialink/20150918-175207/map.txt', 'rb')
# f_handle_open_map = open('/home/avavav/avdata/mobile_base/skygarden1/map.txt', 'rb')

f_handle_open_map = open(os.path.join(dir_prefix, 'map.txt'), 'rb')

data = f_handle_open_map.read()
file = ZipInputStream(StringIO.StringIO(data))
lines = file.readlines()
end = time.time()
print end-start
# for line in lines:
	# print line[:-1]
