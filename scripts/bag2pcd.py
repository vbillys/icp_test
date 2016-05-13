#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import math
from tf import TransformListener


class Bag2UosConverter:
    def __init__(self, rootdir, scale_factor, transform_pitch_angle):
	rospy.init_node('bad_to_uos_convert', anonymous=False)
	rospy.Subscriber('velotime_points', PointCloud2, self.processMsg)
	self.counter = 0
	self.rootdir = rootdir
	self.scale_factor = scale_factor
	self.tf = TransformListener()
	self.init = True
	self.transform_pitch_angle = math.radians(transform_pitch_angle)
	pass
    def transformPoint(self,x ,y , z):
	yy = y
	xx =  math.cos(self.transform_pitch_angle) * x + math.sin(self.transform_pitch_angle) * z
	zz = -math.sin(self.transform_pitch_angle) * x + math.cos(self.transform_pitch_angle) * z
	return xx, yy, zz
    def run(self):
	rospy.spin()
	pass
    def processMsg(self, msg):
	print 'data arrived'
	# print list(cloud)
	# print self.tf.frameExists("velodyne") , self.tf.frameExists("odom")

	if self.tf.frameExists("world") and self.tf.frameExists("odom"):
	    t = self.tf.getLatestCommonTime("world", "odom")
	    position, quat= self.tf.lookupTransform("world", "odom", t)
	    print position, quat
	    euler = tf.transformations.euler_from_quaternion(quat)
	    # heading_rad = math.degrees(euler[2])
	    heading_rad = euler[2]
	    print heading_rad
	    cloud = pc2.read_points(msg, skip_nans=True)
	    self.saveData(list(cloud))
	    self.savePose(position[0],position[1],0,0,0,heading_rad)

	    self.counter = self.counter + 1
	if self.init:
	    self.init = False
	    cloud = pc2.read_points(msg, skip_nans=True)
	    self.saveData(list(cloud))
	    self.savePose(0,0,0,0,0,0)
	    self.counter = self.counter + 1

    def saveData(self, cloud):
	file_string = 'scan'+format(self.counter,'03d')
	fullfilename_data = self.rootdir + file_string + '.3d'
	fh_data  = open(fullfilename_data, 'w')
	for (x,y,z,intensity,ring) in cloud:
	    # print x,y,z
	    x, y, z = self.transformPoint(x, y, z)
	    x = x*self.scale_factor
	    y = y*self.scale_factor
	    z = z*self.scale_factor
	    str_ = format(x, '.1f')    + ' ' + format(z, '.1f')     + ' ' + format(y, '.1f')   + '\n'
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
# g_root_dir = '/home/avavav/Documents/workspace/3dtk/pioneer_fusion_lvl9_try/'
g_root_dir = '/home/avavav/Documents/workspace/3dtk/pioneer_fusion_lvl9_try_3/'
g_scale_factor = 100
g_pitch_angle = 7.5
if __name__ == '__main__':
    converter = Bag2UosConverter(g_root_dir, g_scale_factor, g_pitch_angle)
    converter.run()
