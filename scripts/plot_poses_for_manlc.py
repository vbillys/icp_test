#!/usr/bin/env python

from cython_catkin_example import cython_catkin_example

import math
import numpy as np
# import matplotlib
# matplotlib.use('GTKAgg')
import pylab as plt
import matplotlib.cm as cm
import itertools
import curses


from optparse import OptionParser
import os, sys
from ZipLib import ZipInputStream
from IcpTestTools import *

opt_parser = OptionParser()
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
opt_parser.add_option('-c','--axeslimit', dest='plot_axes_limit', nargs=4 , type='float')
opt_parser.add_option('--ii', dest='indices',  action='append', type='int')
opt_parser.add_option('--mp', dest='mod_pose',  action='append', type='float', nargs=4)
opt_parser.add_option('--rp', dest='recorrect_pose',  action='append', type='int', nargs=2)
opt_parser.add_option('--ap', dest='additive_mod_pose',  action='store_true', default=False)
opt_parser.add_option('-a', dest='show_anim',  action='store_true', default=False)
opt_parser.add_option('--ns', dest='not_static',  action='store_true', default=False)
opt_parser.add_option('--pnm', dest='not_plot_map',  action='store_true', default=False)
opt_parser.add_option('--ua', dest='use_accumulated',  action='store_true', default=False)
opts, args = opt_parser.parse_args(sys.argv[1:])
dir_prefix = opts.dir_prefix


# def plotScan(no, ccscan, scatter):
def plotScan(no, ccscan, ax, color='magenta', size=0.3):
	xs, ys = ccscan.getCloudTransformedFromIndex(no,separate_axes=True)
	# print xs,ys
	ax.scatter (xs,ys, color=color, s=size)
	# ax.scatter (xs,ys,  s=size)
	# scatter.set_offsets(np.column_stack((xs, ys)))

def plotMap(ccscan, ax, size=1.3, color='green'):
	xs, ys = ccscan.getMap(separate_axes=True)
	ax.scatter(xs, ys, color=color, s=size) 

# if opts.use_graph:
	# f_handle2 = open(os.path.join(dir_prefix , 'icp_poses.graph'),'r')
# else:
# f_handle = open(os.path.join(dir_prefix , 'icp_poses.txt'),'r')
# points = getFloatNumberFromReadLines(f_handle, 12)
if opts.show_anim:
	ccscan = ICPScan(dir_prefix, load_map=not opts.not_static, for_anim = True, axis_limit=opts.plot_axes_limit, use_accumulated=opts.use_accumulated)
else:
	ccscan = ICPScan(dir_prefix, load_map=not opts.not_static, use_accumulated=opts.use_accumulated)

# print opts.mod_pose
opt_additive_pose = False
if opts.additive_mod_pose is not None:
	opt_additive_pose = opts.additive_mod_pose
if opts.mod_pose is not None:
	for mp in opts.mod_pose:
		ccscan.modPose(int(mp[0]), [mp[1], mp[2], math.radians(mp[3])], additive_pose = opt_additive_pose)
if opts.recorrect_pose is not None:
	for rp in opts.recorrect_pose:
		ccscan.recorrectPoseUseICP(rp[0],rp[1])
		print ccscan.createEdgeString(rp[0],rp[1])
if not opts.not_static and opts.indices is not None:
	fig, ax = plt.subplots()
	if opts.plot_axes_limit is not None:
		ax.axis(opts.plot_axes_limit)
	ax.set_aspect('equal','datalim')
	if not opts.not_plot_map:
		plotMap(ccscan, ax)
	colors = cm.rainbow(np.linspace(0, 1, len(opts.indices)))
	_index = 0
	for l in opts.indices:
		plotScan(l, ccscan, ax, color=colors[_index])
		print ccscan.getPose(l)
		_index = _index + 1
	plt.show(True)



if opts.show_anim:
	ccscan.setAnimReady()
	plt.show()

