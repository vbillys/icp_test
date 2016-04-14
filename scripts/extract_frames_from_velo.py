#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import math
from tf import TransformListener

class ExtractFramesFromVelodyne:
    def __init__(self, rootdir, scale_factor, sample_freq):
		rospy.init_node('extract_frame_from_velo_node', anonymous=False)
		rospy.Subscriber('velodyne_points', PointCloud2, self.processMsg)
		self.counter = 0
		self.scan_counter =0
		self.rootdir = rootdir
		self.scale_factor = scale_factor
		self.sample_freq = sample_freq
		self.tf = TransformListener()
		self.init = True
		pass
    def run(self):
	rospy.spin()
	pass
    def processMsg(self, msg):
	print 'data arrived'
	# print list(cloud)
	# print self.tf.frameExists("velodyne") , self.tf.frameExists("odom")

	if self.scan_counter % self.sample_freq == 0:
		cloud = pc2.read_points(msg, skip_nans=True)
		self.saveData(list(cloud))
		self.counter = self.counter + 1
	self.scan_counter = 1 + self.scan_counter
	# if self.tf.frameExists("world") and self.tf.frameExists("odom"):
		# t = self.tf.getLatestCommonTime("world", "odom")
	    # position, quat= self.tf.lookupTransform("world", "odom", t)
	    # print position, quat
	    # euler = tf.transformations.euler_from_quaternion(quat)
	    # # heading_rad = math.degrees(euler[2])
	    # heading_rad = euler[2]
	    # print heading_rad
	    # cloud = pc2.read_points(msg, skip_nans=True)
	    # self.saveData(list(cloud))
	    # self.savePose(position[0],position[1],0,0,0,heading_rad)

	    # self.counter = self.counter + 1
	# if self.init:
		# self.init = False
	    # cloud = pc2.read_points(msg, skip_nans=True)
	    # self.saveData(list(cloud))
	    # self.savePose(0,0,0,0,0,0)
	    # self.counter = self.counter + 1

    def saveData(self, cloud):
	file_string = 'scan'+format(self.counter,'03d')
	fullfilename_data = self.rootdir + file_string + '.3d'
	fh_data  = open(fullfilename_data, 'w')
	for (x,y,z,intensity,ring) in cloud:
		# print x,y,z
		x = x*self.scale_factor
		y = y*self.scale_factor
		z = z*self.scale_factor
		str_ = format(x, '.3f')    + ' ' + format(y, '.3f')     + ' ' + format(z, '.3f') + ' '+ format(ring,'d')    + '\n'
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
	str_first_line = format(x, '.1f')    + ' ' + format(-z, '.1f')     + ' ' + format(y, '.1f')   + '\n'
	str_2nd_line   = format(roll, '.1f') + ' ' + format(-yaw, '.1f') + ' ' + format(pitch, '.1f') + '\n'
	print str_first_line+str_2nd_line
	fh_pose.writelines([str_first_line, str_2nd_line])
	fh_pose.close()


# g_root_dir = '/home/avavav/Documents/workspace/3dtk/velo16/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/jtc3/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/jtcpark2/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/infuse2/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/infuse/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/infuse1/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/connexis_set1/'
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/skygarden_set1/'
# g_root_dir = '/home/avavav/avdata/mobile_base/skygarden_set1/'
g_root_dir = '/home/avavav/avdata/mobile_base/skygarden_set2/'
g_scale_factor = 1
if __name__ == '__main__':
	converter = ExtractFramesFromVelodyne(g_root_dir, g_scale_factor, 10)
	converter.run()
