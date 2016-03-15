#!/usr/bin/env python


from cython_catkin_example import cython_catkin_example

import pylab as plt
import itertools
f_handle = open('/home/avavav/avdata/alphard/medialink/set1/icp_poses.txt','r')
f_handle_w = open('icp_poses.graph','w')

example = cython_catkin_example.PyCCExample()

def grouper(n, iterable, fillvalue=None):
	"grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
	args = [iter(iterable)] * n
	return itertools.zip_longest(*args, fillvalue=fillvalue)

f_content = f_handle.readlines()
points = []
for str_ in f_content:
	strs = str_.strip()
	point_ = map(float, strs.split())
	point = zip(*(iter(point_),) * 12)
	points.append(point)
# print points
str_vertex = 'VERTEX2 0 0 0 0\n'
index_point = 1
for vertex in points:
	str_vertex = str_vertex + 'VERTEX2 '
	str_vertex = str_vertex + format(index_point,'d') + ' '
	str_vertex = str_vertex + format(vertex[0][0],'.4f') + ' '
	str_vertex = str_vertex + format(vertex[0][1],'.4f') + ' '
	str_vertex = str_vertex + format(vertex[0][2],'.4f')
	str_vertex = str_vertex + '\n'
	index_point = index_point + 1


str_edge = ''
index_point = 0
for edge in points:
	str_edge = str_edge + 'EDGE2 '
	str_edge = str_edge + format(index_point+1,'d') + ' '
	str_edge = str_edge + format(index_point  ,'d') + ' '
	str_edge = str_edge + format(edge[0][3],'.4f') + ' '
	str_edge = str_edge + format(edge[0][4],'.4f') + ' '
	str_edge = str_edge + format(edge[0][5],'.4f') + ' '
	str_edge = str_edge + format(edge[0][6],'.4f') + ' '
	str_edge = str_edge + format(edge[0][7],'.4f') + ' '
	str_edge = str_edge + format(edge[0][8],'.4f') + ' '
	str_edge = str_edge + format(edge[0][9],'.4f') + ' '
	str_edge = str_edge + format(edge[0][10],'.4f') + ' '
	str_edge = str_edge + format(edge[0][11],'.4f')
	# str_edge = str_edge + '1 0 1 1 0 0'
	str_edge = str_edge + '\n'
	index_point = index_point + 1

f_handle_w.write(str_vertex)
f_handle_w.write(str_edge)
