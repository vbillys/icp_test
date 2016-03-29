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
import curses
import pylab as plt
import matplotlib.animation as animation

from scipy.spatial import KDTree


from optparse import OptionParser
import os, sys


opt_parser = OptionParser()
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('--lc', dest='loop_closure',  action='store_true', default=False)
opt_parser.add_option('--uc', dest='use_accumulated',  action='store_true', default=False)
opt_parser.add_option('--nf', dest='icp_use_filtered_data',  action='store_false', default=True)

opt_parser.add_option('-s','--start', dest='start_index',type='int')
opt_parser.add_option('-e','--end', dest='end_index', type='int')
opts, args = opt_parser.parse_args(sys.argv[1:])
start_index = 0
end_index = None
if opts.start_index is not None:
	start_index = int(opts.start_index)
if opts.end_index is not None:
	end_index = int(opts.end_index)
dir_prefix = opts.dir_prefix
print  opts, args

# dir_prefix = '/home/avavav/avdata/alphard/onenorth/onenorth_wb_2016-03-24-10-05-56/'

# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-174721/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-175207/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_lm_poses.txt','r')

# f_handle = open('/home/avavav/avdata/mobile_base/skygarden1/icp_poses.txt','r')

# f_handle = open('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/onenorth/20150821-120036_sss/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/onenorth/20150821-115715_sss/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/onenorth/20150821-115401_sss/icp_poses.txt','r')
# f_handle = open('/home/avavav/avdata/alphard/onenorth/20150821-115223_sss/icp_poses.txt','r')


if not opts.use_accumulated:
	f_handle = open(os.path.join(dir_prefix , 'icp_poses.txt'),'r')
	f_handle_w = open(os.path.join(dir_prefix ,'icp_poses.graph'),'w')
else:
	f_handle = open(os.path.join(dir_prefix , 'icp_lm_poses.txt'),'r')
	f_handle_w = open(os.path.join(dir_prefix ,'icp_lm_poses.graph'),'w')
# f_handle_w = open('icp_poses_lm.graph','w')


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


def grouper(n, iterable, fillvalue=None):
	"grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
	args = [iter(iterable)] * n
	return itertools.zip_longest(*args, fillvalue=fillvalue)

points = getFloatNumberFromReadLines(f_handle, 12)
points_2d = [[p[0],p[1]] for p in points]
if end_index is None:
	end_index = len(points)
points = points[start_index:end_index]
points_2d = points_2d [start_index:end_index]
tree_points  = KDTree(np.array(points_2d), leafsize=30)

rospy.init_node('point_cloud_pub_from_process_graph_node', anonymous=False)
g_pub_ros = rospy.Publisher("ibeo_points", PointCloud2)
def publishScan(no):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'ibeo'
	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	H_points = [[p[0], p[1],1] for p in test_cloud]
	pc_point_t = pc2.create_cloud_xyz32(header, H_points)
	g_pub_ros.publish(pc_point_t)

# rospy.wait_for_service('processICP')
g_processICP_srv = rospy.ServiceProxy('processICP', processICP)
def computeICPBetweenScans(no1,no2, init_x = 0 , init_y = 0, init_yaw = 0):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'ibeo'

	example = cython_catkin_example.PyCCExample()

	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no1)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(no1)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no1)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(no1)+'.txt')


	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/scan_filtered_'+str(no1)+'.txt')

	if opts.use_accumulated:
		test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix,'scan_lm_filtered_'+str(no1)+'.txt'))
		if opts.icp_use_filtered_data:
			test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix,'scan_filtered_'+str(no1)+'.txt'))
		else:
			test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix,'scan_'+str(no1)+'.txt'))

	H_points = [[p[0], p[1],1] for p in test_cloud]
	# H_points = [[np.float32(p[0]), np.float32(p[1]),1] for p in test_cloud]
	pc_point_t1 = pc2.create_cloud_xyz32(header, H_points)
	# print 'text1: ',len(test_cloud)
	# print test_cloud
	# example.load_2d_array('ref_map',np.array(test_cloud, np.float32)) 

	# test_cloud2 = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no2)+'.txt')
	# test_cloud2 = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(no2)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no2)+'.txt')
	# test_cloud2 = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(no2)+'.txt')


	# test_cloud2 = read2DPointsFromTextFile('/home/avavav/avdata/alphard/onenorth/20150821-114839_sss/scan_filtered_'+str(no2)+'.txt')

	if opts.use_accumulated:
		test_cloud = read2DPointsFromTextFile(os.path.join(dir_prefix,'scan_lm_filtered_'+str(no2)+'.txt'))
		if opts.icp_use_filtered_data:
			test_cloud2 = read2DPointsFromTextFile(os.path.join(dir_prefix,'scan_filtered_'+str(no2)+'.txt'))
		else:
			test_cloud2 = read2DPointsFromTextFile(os.path.join(dir_prefix,'scan_'+str(no2)+'.txt'))

	H_points = [[p[0], p[1],1] for p in test_cloud2]
	# H_points = [[np.float32(p[0]), np.float32(p[1]),1] for p in test_cloud]
	pc_point_t2 = pc2.create_cloud_xyz32(header, H_points)
	# print 'text2: ',len(test_cloud)
	# example.load_2d_array('que_map',np.array(test_cloud, np.float32)) 
	# names = example.get_point_xyz_clouds_names()
	# print("Current point clouds: " + ",".join(names))

	# icp_ros_result = g_processICP_srv(pc_point_t1, pc_point_t2)
	# print icp_ros_result


	# cython_icp_result = example.processICP(np.array(test_cloud, np.float32),np.array(test_cloud2, np.float32))
	cython_icp_result = example.processICP(np.array([x[0] for x in test_cloud], np.float32),np.array([x[1] for x in test_cloud], np.float32),np.array([x[0] for x in test_cloud2], np.float32),np.array([x[1] for x in test_cloud2], np.float32), init_x, init_y, init_yaw)
	# print cython_icp_result
	return cython_icp_result

	# return icp_ros_result.result.pose.pose.position.x, icp_ros_result.result.pose.pose.position.y, icp_ros_result.result.pose.pose.position.z,icp_ros_result.result.pose.covariance[0],icp_ros_result.result.pose.covariance[1],icp_ros_result.result.pose.covariance[2],icp_ros_result.result.pose.covariance[3],icp_ros_result.result.pose.covariance[4],icp_ros_result.result.pose.covariance[5]

# example = cython_catkin_example.PyCCExample()
# # test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_6.txt')
# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_5.txt')
# # print test_cloud
# example.load_2d_array('ref_map',np.array(test_cloud, np.float32)) 
# # test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_7.txt')
# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_6.txt')
# example.load_2d_array('que_map',np.array(test_cloud, np.float32)) 
# names = example.get_point_xyz_clouds_names()
# print("Current point clouds: " + ",".join(names))
# # example.processICP(points[1][3],points[1][4],points[1][5])
# print example.processICP()
# print points[5]#[3:6]
# print points[6]#[3:6]
# print points[7]#[3:6]
# print points[8]#[3:6]


str_vertex = 'VERTEX2 0 0 0 0\n'
index_point = 1
for vertex in points:
	str_vertex = str_vertex + 'VERTEX2 '
	str_vertex = str_vertex + format(index_point,'d') + ' '
	str_vertex = str_vertex + format(vertex[0],'.4f') + ' '
	str_vertex = str_vertex + format(vertex[1],'.4f') + ' '
	str_vertex = str_vertex + format(vertex[2],'.4f')
	str_vertex = str_vertex + '\n'
	index_point = index_point + 1


def transformCloud(test_cloud, tm):
	H_points = [[p[0], p[1],1] for p in test_cloud]
	M_points = np.matrix(H_points)
	point_t = tm *M_points.getT()
	point_t = point_t.getT().tolist()
	return point_t	

def plotScan(no, ax):
	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	# return ax.scatter ([x[0] for x in test_cloud],[x[1] for x in test_cloud], color='blue', s=8)
	ax.set_offsets(np.column_stack(([x[0] for x in test_cloud],[x[1] for x in test_cloud])))
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no)+'.txt')
	# ax.scatter ([x[0] for x in test_cloud],[x[1] for x in test_cloud], color='red', s=8)


def plotScanTransformed(no, ax, tm):
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(no)+'.txt')

	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/mobile_base/skygarden1/scan_'+str(no)+'.txt')

	point_t = transformCloud(test_cloud, tm)
	ax.set_offsets(np.column_stack(([x[0] for x in point_t],[x[1] for x in point_t])))

def plotMapTransformed(no, ax, tm, map_xx, map_yy):
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_filtered_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no+1)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(no)+'.txt')

	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/mobile_base/skygarden1/scan_filtered_'+str(no)+'.txt')

	point_t = transformCloud(test_cloud, tm)
	map_xx = map_xx + [x[0] for x in point_t]
	map_yy = map_yy + [x[1] for x in point_t]
	
	ax.set_offsets(np.column_stack((map_xx,map_yy))) 
	return map_xx, map_yy

def accumulateIcpTransform(icp_result, x, y, yaw):
	x_icp =  math.cos(-icp_result[2])*icp_result[0] + math.sin(-icp_result[2])*icp_result[1] 
	y_icp = -math.sin(-icp_result[2])*icp_result[0] + math.cos(-icp_result[2])*icp_result[1]
	x_icp_g =  math.cos(-yaw)*x_icp + math.sin(-yaw)*y_icp + x
	y_icp_g = -math.sin(-yaw)*x_icp + math.cos(-yaw)*y_icp + y
	yaw_icp_g = yaw + icp_result[2]
	return x_icp_g, y_icp_g, yaw_icp_g

# ax = plt.axes()
# plotScan(0, ax)
# ax.scatter(points_tracked[0], points_tracked[1] , color='blue', s=8)
g_thresh = 10. #.5 #10.
class AnimatedScatter(object):
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		# self.ax.axis([-200, 100, -100, 200])
		self.ax.axis([-50, 50, -50, 50])
		self.ax.set_aspect('equal','datalim')
		# self.no_frame = 0
		self.points_map_x = []
		self.points_map_y = []
		self.travelled_dist = 0 
		self.last_x = 0
		self.last_y = 0
		self.last_yaw = 0
		self.next_capture_dist = 0- g_thresh
		self._to_clear_2 = None
		self.ani = animation.FuncAnimation(self.fig, self.update, interval=300, init_func=self.setup_plot, blit=True, frames=len(points)-1, repeat=False)
		# self.ani = animation.FuncAnimation(self.fig, self.update, interval=80, init_func=self.setup_plot, blit=True, frames=2000, repeat=False)
		# self.ani = animation.FuncAnimation(self.fig, self.update, interval=80,  blit=True, frames=len(points)-1, repeat=False)
	def setup_plot(self):
		print 'hi setup anim'
		self.scatter_scan = self.ax.scatter ([],[], color='blue', s=8)
		self.scatter_map = self.ax.scatter ([],[], color='green', s=.4)
		return self.scatter_scan, self.scatter_map
	def update(self,i):
		print 'updating figure...', i
		# print self.no_frame
		# _to_clear = plotScan(self.no_frame)
		# print computeICPBetweenScans(i,i+1)
		# print points[i+1]
		# publishScan(i)

		print points[i]
		yaw = points[i][2]
		x = points[i][0]
		y = points[i][1]
		# if i == 0:
			# yaw = 0
			# x = 0
			# y = 0
		# else:
			# icp_result = computeICPBetweenScans(i-1,i)
			# x,y,yaw = accumulateIcpTransform(icp_result,self.last_x,self.last_y,self.last_yaw)
		self.travelled_dist = self.travelled_dist + math.hypot(self.last_x - x, self.last_y - y)
		# print self.travelled_dist, self.next_capture_dist
		self.last_x = x
		self.last_y = y
		self.last_yaw = yaw

		tranformation_matrix = np.matrix([[math.cos(-yaw),math.sin(-yaw),x],[-math.sin(-yaw),math.cos(-yaw),y],[0,0,1]])
		_to_clear = [self.scatter_scan]
		# _to_clear.append(plotScan(i, self.scatter_scan))
		# plotScan(i, self.scatter_scan)
		plotScanTransformed(i, self.scatter_scan, tranformation_matrix)
		if self.travelled_dist > self.next_capture_dist:
			while self.travelled_dist > self.next_capture_dist:
				self.next_capture_dist = self.next_capture_dist + g_thresh
			# if self._to_clear_2 is not None:
			self.points_map_x, self.points_map_y = plotMapTransformed(i, self.scatter_map, tranformation_matrix, self.points_map_x, self.points_map_y)
			# self._to_clear_2 = _to_clear_2
		else:
			self.scatter_map.set_offsets(np.column_stack((self.points_map_x, self.points_map_y)))
			# self.no_frame = self.no_frame + 1
			# return _to_clear_1, _to_clear_2
		_to_clear = _to_clear + [self.scatter_map]
		return _to_clear

# AnimatedScatter()
# plt.show()
# exit()



# str_edge_recomputed = ''
# index_point = 0
# for edge in points:
	# str_edge_recomputed = str_edge_recomputed + 'EDGE2 '
	# str_edge_recomputed = str_edge_recomputed + format(index_point+1,'d') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(index_point  ,'d') + ' '
	# myscreen.addstr(12,25,'Recomputing ICP, %(index_point)d/%(total_points)d' % {'index_point':index_point,'total_points':len(points)})
	# myscreen.refresh()
	# recomputed = computeICPBetweenScans(index_point, index_point+1)
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[0],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[1],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[2],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[3],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[4],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[5],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[6],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[7],'.4f') + ' '
	# str_edge_recomputed = str_edge_recomputed + format(recomputed[8],'.4f')
	# str_edge_recomputed = str_edge_recomputed + '\n'
	# index_point = index_point + 1


str_edge = ''
index_point = 0
for edge in points:
	str_edge = str_edge + 'EDGE2 '
	str_edge = str_edge + format(index_point+1,'d') + ' '
	str_edge = str_edge + format(index_point  ,'d') + ' '
	if index_point == 0 and start_index > 0:
		str_edge = str_edge + format(edge[0],'.4f') + ' '
		str_edge = str_edge + format(edge[1],'.4f') + ' '
		str_edge = str_edge + format(edge[2],'.4f') + ' '
	else:
		str_edge = str_edge + format(edge[3],'.4f') + ' '
		str_edge = str_edge + format(edge[4],'.4f') + ' '
		str_edge = str_edge + format(edge[5],'.4f') + ' '
	str_edge = str_edge + format(edge[6],'.4f') + ' '
	str_edge = str_edge + format(edge[7],'.4f') + ' '
	str_edge = str_edge + format(edge[8],'.4f') + ' '
	str_edge = str_edge + format(edge[9],'.4f') + ' '
	str_edge = str_edge + format(edge[10],'.4f') + ' '
	str_edge = str_edge + format(edge[11],'.4f')
	# str_edge = str_edge + '1 0 1 1 0 0'
	str_edge = str_edge + '\n'
	index_point = index_point + 1

# print str_vertex
# print str_edge
f_handle_w.write(str_vertex)
f_handle_w.write(str_edge)
# f_handle_w.write(str_edge_recomputed)

if not opts.loop_closure:
	exit()

# find 10 closest vertices for every vertex
# also test if the icp result is bad, if good only add the EDGE constraint

def getInitialValues(no1,no2):
	no1_pose = points[no1]
	no2_pose = points[no2]
	x = no2_pose[0] - no1_pose[0]
	y = no2_pose[1] - no1_pose[1]
	yaw = no2_pose[2] - no1_pose[2]
	return x,y,yaw


myscreen = curses.initscr()
myscreen.addstr(12,25,'Recomputing ICP, 0/%(total_points)d' % {'total_points':len(points)})
myscreen.refresh()

str_edge_added = ''
index_point = 0
for vertex in points_2d:
	dist, ind = tree_points.query(vertex, k=41) #21
	threshold_ind  = []
	threshold_dist = []

	no_of_added_edge = 0
	for idist, iind in zip (dist, ind):
		# if idist > 2.5 and idist < 10. and abs(index_point - iind) > 10:# > 2.5 and idist < 4.0:
		if idist < 30. and abs(index_point - iind) > 10:# > 2.5 and idist < 4.0:
		# if idist > 2.5 and idist < 10:# > 2.5 and idist < 4.0:
			threshold_ind.append(iind)
			threshold_dist.append(idist)
			no_of_added_edge = no_of_added_edge + 1
		if no_of_added_edge > 8:#8:
			break

	# print vertex, index_point, threshold_ind, threshold_dist
	for idist, iind in zip (threshold_dist, threshold_ind):
		init_val = getInitialValues(index_point, iind)
		icp_edge = computeICPBetweenScans(index_point+1, iind+1, init_val[0], init_val[1], init_val[2])
		# icp_edge = computeICPBetweenScans(index_point, iind)
		# icp_edge = computeICPBetweenScans(index_point, iind+1)
		myscreen.addstr(12,25,'Recomputing ICP, %(index_point)d/%(total_points)d' % {'index_point':index_point+2,'total_points':len(points)})
		myscreen.refresh()
		if icp_edge[9] > .7: #0.8: #0.82:
			str_edge_added = str_edge_added + 'EDGE2 '
			str_edge_added = str_edge_added + format(iind+1,'d') + ' '
			str_edge_added = str_edge_added + format(index_point+1,'d') + ' '
			str_edge_added = str_edge_added + format(icp_edge[0],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[1],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[2],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[3],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[4],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[5],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[6],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[7],'.4f') + ' '
			str_edge_added = str_edge_added + format(icp_edge[8],'.4f') + ' '
			str_edge_added = str_edge_added + '\n'
	# print index_point, len(threshold_ind) 
	index_point = index_point + 1

f_handle_w.write(str_edge_added)
curses.endwin()
