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
import matplotlib.animation as animation

from scipy.spatial import KDTree

import IcpTestTools

from optparse import OptionParser
import os, sys

rospy.init_node('local_map_server_node', anonymous=False)
g_pub_map = rospy.Publisher("icp_globalmap", PointCloud2, latch = True)

opt_parser = OptionParser()
opt_parser.add_option('-d','--directory', dest='dir_prefix', type='string', default='')
# opt_parser.add_option('--uc', dest='use_accumulated',  action='store_true', default=False)
# opt_parser.add_option('--nf', dest='icp_use_filtered_data',  action='store_false', default=True)
# opt_parser.add_option('--oc', dest='override_covariances',  action='store_true', default=False)
# opt_parser.add_option('--pm', dest='use_processed_map',  action='store_true', default=False)
# opt_parser.add_option('--rpm', dest='reverse_processed_map',  action='store_true', default=False)
# opt_parser.add_option('--ogifn', dest='initial_graph_filename', type='string', default=None)
# opt_parser.add_option('--ogffn', dest='final_graph_filename', type='string', default=None)
# opt_parser.add_option('-g','--gthresh', dest='g_thresh', type='float', default=10.)

opts, args = opt_parser.parse_args(sys.argv[1:])

dir_prefix = opts.dir_prefix
print  opts, args

map_obj = IcpTestTools.ICPScan(dir_prefix, load_map = True )
# xmap, ymap = map_obj.getMap(separate_axes = True)
# fig, ax = plt.subplots()
# ax.scatter(xmap, ymap, color='green', s=.3)
# plt.show(True)

wmap = map_obj.getMap()
wmap = [list(x) for x in wmap]
wmap = [x + [0] for x in wmap]
# print wmap

# tree_points  = KDTree(np.array(wmap), leafsize=30)


def talker():
  header = Header()
  header.stamp = rospy.Time.now()
  header.frame_id = 'map'
  g_pcloud = pc2.create_cloud_xyz32(header, wmap)
  g_pub_map.publish(g_pcloud)
  rospy.spin()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
