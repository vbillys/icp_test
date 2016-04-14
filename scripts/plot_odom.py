#!/usr/bin/env python
import pylab as plt
import itertools
# f_handle = open('odom_log_infuse.csv','r')
# f_handle = open('odom_log_infuse_1.csv','r')
# f_handle = open('odom_log_infuse_2.csv','r')
f_handle = open('/home/avavav/avdata/alphard/medialink/set1/icp_poses.txt','r')

def plotPoints(data):
	ax = plt.axes()
	xs, ys = [], []
	for point in data:
		xs.append(point[0][0])
		ys.append(point[0][1])

	# scat = ax.scatter(xs, ys , color='b', s=24, marker='+')
	# plt.plot(xs, ys , 'ro-', lw=1)
	plt.plot(xs, ys , 'r--', lw=1)

	ax.xaxis.grid(True, which="major", linestyle='dotted')
	ax.yaxis.grid(True, which="major", linestyle='dotted')
	ax.axis('equal')
	plt.show()

def grouper(n, iterable, fillvalue=None):
	"grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
	args = [iter(iterable)] * n
	return itertools.zip_longest(*args, fillvalue=fillvalue)

f_content = f_handle.readlines()
points = []
for str_ in f_content:
	strs = str_.strip()
	point_ = map(float, strs.split())
	point = zip(*(iter(point_),) * 3)
	points.append(point)
# print points

plotPoints(points)
