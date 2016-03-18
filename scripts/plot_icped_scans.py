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


f_handle_pose = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.txt','r')
# f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses.graph','r')
# f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-initial.graph','r')
# f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses-treeopt-final.graph','r')
f_handle2 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm.graph','r')
f_handle = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm-treeopt-initial.graph','r')
f_handle3 = open('/home/avavav/avdata/alphard/medialink/20150918-180619/icp_poses_lm-treeopt-final.graph','r')


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
			coord = [-float(strs_splitted[2]), -float(strs_splitted[3]), -float(strs_splitted[4])]
		else:
			coord = [float(strs_splitted[2]), float(strs_splitted[3]), float(strs_splitted[4])]
		points.append([noid]+coord)
	return points

ng = 43 # 2433

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
plt.plot([o[1] for o in vertices],[-o[2] for o in vertices])
plt.plot([o[1] for o in vertices2],[o[2] for o in vertices2])
plt.plot([o[1] for o in vertices3],[-o[2] for o in vertices3])
# # plt.plot([o[0] for o in points],[o[1] for o in points])
for vertex in vertices3:
# for point in points:
	if checkpoints % 25 == 0:
		test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(vertex[0])+'.txt')
		# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(vertex[0])+'.txt')
		# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(checkpoints)+'.txt')
		t_point_t = transformCloud(test_cloud, createTransfromFromXYYaw(vertex[1],vertex[2],-vertex[3]))
		# t_point_t = transformCloud(test_cloud, createTransfromFromXYYaw(point[0],point[1],-point[2]))
		point_t = point_t + t_point_t
	checkpoints = checkpoints + 1
ax.scatter ([x[0] for x in point_t],[x[1] for x in point_t], color='green', s=.3)
# fig.canvas.draw()
plt.show(True)


