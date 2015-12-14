#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import math


class Bag2UosConverter:
    def __init__(self, rootdir, scale_factor):
	rospy.init_node('bad_to_uos_convert', anonymous=False)
	rospy.Subscriber('velodyne_points', PointCloud2, self.processMsg)
	self.counter = 0
	self.rootdir = rootdir
	self.scale_factor = scale_factor
	pass
    def run(self):
	rospy.spin()
	pass
    def processMsg(self, msg):
	print 'data arrived'
	cloud = pc2.read_points(msg, skip_nans=True)
	# print list(cloud)
	self.saveData(list(cloud))
	self.savePose(0,0,0,0,0,0)
	self.counter = self.counter + 1
    def saveData(self, cloud):
	file_string = 'scan'+format(self.counter,'03d')
	fullfilename_data = self.rootdir + file_string + '.3d'
	fh_data  = open(fullfilename_data, 'w')
	for (x,y,z,intensity,ring) in cloud:
	    # print x,y,z
	    x = x*self.scale_factor
	    y = y*self.scale_factor
	    z = z*self.scale_factor
	    str_ = format(x, '.1f')    + ' ' + format(-z, '.1f')     + ' ' + format(y, '.1f')   + '\n'
	    fh_data.write(str_)
	fh_data.close()

    def savePose(self, x, y, z, roll, pitch, yaw):
	file_string = 'scan'+format(self.counter,'03d')
	fullfilename_pose = self.rootdir + file_string + '.pose'
	fh_pose  = open(fullfilename_pose, 'w')
	x = x*self.scale_factor
	y = y*self.scale_factor
	z = z*self.scale_factor
	roll  = roll*180/math.pi
	yaw   = yaw*180/math.pi
	pitch = pitch*180/math.pi
	str_first_line = str(x)    + ' ' + str(-z)     + ' ' + str(y)   + '\n'
	str_2nd_line   = str(roll) + ' ' + str(-yaw) + ' ' + str(pitch) + '\n'
	print str_first_line+str_2nd_line
	fh_pose.writelines([str_first_line, str_2nd_line])
	fh_pose.close()


g_root_dir = '/home/avavav/Documents/workspace/3dtk/velo16/'
g_scale_factor = 100
if __name__ == '__main__':
    converter = Bag2UosConverter(g_root_dir, g_scale_factor)
    converter.run()
