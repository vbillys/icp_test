#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from roslib import message

import xyz
import re
import sensor_msgs.point_cloud2 as pc2
from roslib import message

rospy.init_node('point_cloud_repub', anonymous=False)
pub_cloud = rospy.Publisher("filtered_points", PointCloud2)

def filter_points(msg):
    print 'got point cloud...'
    #pcloud = PointCloud2()
    #pub_cloud.publish(pcloud)
    #pub_cloud.publish(msg)
    print msg.header
    print msg.height
    print msg.width
    print msg.fields
    print msg.is_bigendian
    print msg.point_step
    print msg.row_step
    print msg.is_dense

g_data_dir = '/home/avavav/Documents/workspace/3dtk/'
g_scale_factor = 0.01

def read_xyz(fh):
    content = fh.readlines()
    cloud = []
    for s in content:
	s_parsed = re.findall( r'\d+\.*\d*', s)
	# s_parsed_floated = []
	# for _f in s_parsed:
	    # s_parsed_floated.append(float(_f)*.1)

	# cloud.append(s_parsed_floated)
	cloud.append([float(s_parsed[0])*g_scale_factor,float(s_parsed[2])*g_scale_factor,float(s_parsed[1])*g_scale_factor])

	# print s, s_parsed, s_parsed_floated

    # return s_parsed_floated
    return cloud


def publish_xyz (filename):
    fullfilename = g_data_dir + filename
    fh = open(fullfilename, 'r')
    cloud = read_xyz(fh)
    pcloud = PointCloud2()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'velodyne'
    
    pcloud = pc2.create_cloud_xyz32(header, cloud)
    pub_cloud.publish(pcloud)
    # print pcloud



def talker():

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

	publish_xyz ('scan000.txt')
	rate.sleep()

    # rospy.spin()


rospy.Subscriber('velodyne_points', PointCloud2, filter_points)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
