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
# import curses
import pylab as plt
import matplotlib.animation as animation

f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.txt','r')
f_handle_w = open('icp_poses.graph','w')


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

rospy.wait_for_service('processICP')
g_processICP_srv = rospy.ServiceProxy('processICP', processICP)
def computeICPBetweenScans(no1,no2):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'ibeo'

	example = cython_catkin_example.PyCCExample()

	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no1)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no1)+'.txt')

	H_points = [[p[0], p[1],1] for p in test_cloud]
	pc_point_t1 = pc2.create_cloud_xyz32(header, H_points)
	# print 'text1: ',len(test_cloud)
	# print test_cloud
	example.load_2d_array('ref_map',np.array(test_cloud, np.float32)) 

	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no2)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no2)+'.txt')

	H_points = [[p[0], p[1],1] for p in test_cloud]
	pc_point_t2 = pc2.create_cloud_xyz32(header, H_points)
	# print 'text2: ',len(test_cloud)
	example.load_2d_array('que_map',np.array(test_cloud, np.float32)) 
	# names = example.get_point_xyz_clouds_names()
	# print("Current point clouds: " + ",".join(names))
	icp_ros_result = g_processICP_srv(pc_point_t1, pc_point_t2)
	# print icp_ros_result
	# return example.processICP()
	return icp_ros_result.result.pose.pose.position.x, icp_ros_result.result.pose.pose.position.y, icp_ros_result.result.pose.pose.position.z,icp_ros_result.result.pose.covariance[0],icp_ros_result.result.pose.covariance[1],icp_ros_result.result.pose.covariance[2],icp_ros_result.result.pose.covariance[3],icp_ros_result.result.pose.covariance[4],icp_ros_result.result.pose.covariance[5]

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
	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no)+'.txt')
	point_t = transformCloud(test_cloud, tm)
	ax.set_offsets(np.column_stack(([x[0] for x in point_t],[x[1] for x in point_t])))

def plotMapTransformed(no, ax, tm, map_xx, map_yy):
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no+1)+'.txt')
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
g_thresh = 10.
class AnimatedScatter(object):
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		self.ax.axis([-200, 100, -100, 200])
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
		self.ani = animation.FuncAnimation(self.fig, self.update, interval=80, init_func=self.setup_plot, blit=True, frames=len(points)-1, repeat=False)
		# self.ani = animation.FuncAnimation(self.fig, self.update, interval=80, init_func=self.setup_plot, blit=True, frames=2000, repeat=False)
		# self.ani = animation.FuncAnimation(self.fig, self.update, interval=80,  blit=True, frames=len(points)-1, repeat=False)
	def setup_plot(self):
		print 'hi setup anim'
		self.scatter_scan = self.ax.scatter ([],[], color='blue', s=8)
		self.scatter_map = self.ax.scatter ([],[], color='green', s=4)
		return self.scatter_scan, self.scatter_map
	def update(self,i):
		print 'updating figure...', i
		# print self.no_frame
		# _to_clear = plotScan(self.no_frame)
		# print computeICPBetweenScans(i,i+1)
		# print points[i+1]
		# publishScan(i)

		# yaw = points[i][2]
		# x = points[i][0]
		# y = points[i][1]
		if i == 0:
			yaw = 0
			x = 0
			y = 0
		else:
			icp_result = computeICPBetweenScans(i-1,i)
			x,y,yaw = accumulateIcpTransform(icp_result,self.last_x,self.last_y,self.last_yaw)
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

AnimatedScatter()
plt.show()
exit()

myscreen = curses.initscr()
myscreen.addstr(12,25,'Recomputing ICP, 0/%(total_points)d' % {'total_points':len(points)})
myscreen.refresh()


str_edge_recomputed = ''
index_point = 0
for edge in points:
	str_edge_recomputed = str_edge_recomputed + 'EDGE2 '
	str_edge_recomputed = str_edge_recomputed + format(index_point+1,'d') + ' '
	str_edge_recomputed = str_edge_recomputed + format(index_point  ,'d') + ' '
	myscreen.addstr(12,25,'Recomputing ICP, %(index_point)d/%(total_points)d' % {'index_point':index_point,'total_points':len(points)})
	myscreen.refresh()
	recomputed = computeICPBetweenScans(index_point, index_point+1)
	str_edge_recomputed = str_edge_recomputed + format(recomputed[0],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[1],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[2],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[3],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[4],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[5],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[6],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[7],'.4f') + ' '
	str_edge_recomputed = str_edge_recomputed + format(recomputed[8],'.4f')
	str_edge_recomputed = str_edge_recomputed + '\n'
	index_point = index_point + 1


str_edge = ''
index_point = 0
for edge in points:
	str_edge = str_edge + 'EDGE2 '
	str_edge = str_edge + format(index_point+1,'d') + ' '
	str_edge = str_edge + format(index_point  ,'d') + ' '
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

f_handle_w.write(str_vertex)
# f_handle_w.write(str_edge)
f_handle_w.write(str_edge_recomputed)

curses.endwin()

