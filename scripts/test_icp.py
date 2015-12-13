#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from roslib import message
import math

import xyz
import re
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import transformation

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

g_data_dir = '/home/avavav/Documents/workspace/3dtk/hannover1/'
g_scale_factor = 0.01

def read_xyz(fh, fh_pose):
    content_pose = fh_pose.readlines()
    pose_trans_parsed = re.findall( r'\d+\.*\d*', content_pose[0])
    pose_rot_parsed   = re.findall( r'\d+\.*\d*', content_pose[1])
    x_pose = float(pose_trans_parsed[2])*g_scale_factor
    y_pose = -float(pose_trans_parsed[0])*g_scale_factor
    heading_deg = float(pose_rot_parsed[1])
    # print pose_rot_parsed, pose_trans_parsed
    print x_pose, y_pose, heading_deg
    Rz = transformation.rotation_matrix(heading_deg*math.pi/180, [0,0,1])
    content = fh.readlines()
    cloud = []
    for s in content:
	s_parsed = re.findall( r'\d+\.*\d*', s)
	# s_parsed_floated = []
	# for _f in s_parsed:
	    # s_parsed_floated.append(float(_f)*.1)

	# cloud.append(s_parsed_floated)

	point  = [float(s_parsed[2])*g_scale_factor,-float(s_parsed[0])*g_scale_factor,float(s_parsed[1])*g_scale_factor]
	point_t = Rz.dot([point[0], point[1], point[2], 0])
	# print point_t
	cloud.append([point_t[0]+x_pose, point_t[1]+y_pose, point_t[2]])

	# print s, s_parsed, s_parsed_floated

    # return s_parsed_floated
    return cloud


def publish_xyz (filename):
    fullfilename_cloud = g_data_dir + filename + '.3d'
    fullfilename_pose  = g_data_dir + filename + '.pose'
    fh = open(fullfilename_cloud, 'r')
    fh_pose = open(fullfilename_pose, 'r')
    cloud = read_xyz(fh, fh_pose)
    pcloud = PointCloud2()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'velodyne'
    
    pcloud = pc2.create_cloud_xyz32(header, cloud)
    pub_cloud.publish(pcloud)
    # print pcloud



def talker():

    rate = rospy.Rate(10)
    counter_index = 0
    while not rospy.is_shutdown():

	file_string = 'scan'+format(counter_index,'03d')
	publish_xyz (file_string)
	print file_string
	counter_index = counter_index + 1
	if counter_index > 468:
	    counter_index = 0
	rate.sleep()

    # rospy.spin()


rospy.Subscriber('velodyne_points', PointCloud2, filter_points)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
