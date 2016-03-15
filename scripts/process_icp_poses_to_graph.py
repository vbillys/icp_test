#!/usr/bin/env python


from cython_catkin_example import cython_catkin_example

import numpy as np
import pylab as plt
import itertools
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

example = cython_catkin_example.PyCCExample()
test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_0.txt')
example.load_2d_array('ref_map',np.array(test_cloud, np.float32)) 
test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_1.txt')
example.load_2d_array('que_map',np.array(test_cloud, np.float32)) 
names = example.get_point_xyz_clouds_names()
print("Current point clouds: " + ",".join(names))
example.processICP()

def grouper(n, iterable, fillvalue=None):
	"grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
	args = [iter(iterable)] * n
	return itertools.zip_longest(*args, fillvalue=fillvalue)

points = getFloatNumberFromReadLines(f_handle, 12)

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
f_handle_w.write(str_edge)
