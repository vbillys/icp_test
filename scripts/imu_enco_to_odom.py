#!/usr/bin/env python
import rospy
import tf
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped


def normalizeHeading(angle_rad):
    while angle_rad < 0:
	angle_rad = angle_rad + math.pi*2
    while angle_rad > 2*math.pi:
	angle_rad = angle_rad - math.pi*2
    return angle_rad

class ImuEnco2Odom:
    def __init__(self, dist_to_meter, dist_thres):
	rospy.init_node('imu_enco_to_odom', anonymous=False)
	rospy.Subscriber('imu/data', Imu, self.processImuMsg)
	rospy.Subscriber('encoder_odom', Odometry, self.processOdomMsg)
	rospy.Subscriber('velodyne_points', PointCloud2, self.processVeloMsg)
	self.repub_velo = rospy.Publisher('velotime_points', PointCloud2)
	self.dist_to_meter = dist_to_meter
	self.dist_thres_for_adding_data = dist_thres
	self.heading_rad = 0
	self.x_pose = 0
	self.y_pose = 0
	self.dist   = 0
	self.dist_accum   = 0
	self.dist_accum_last = 0
	self.last_odom = 0
	self.init = False
	self.init_velo = True
	self.pub_br = tf.TransformBroadcaster()

    def run(self):
	rospy.spin()
    def processVeloMsg(self, msg):
	# self.velostamp = msg.header.stamp
	# self.init_velo = True

	# trans_velo = TransformStamped()
	# trans_velo.stamp = rospy.Time.now()
	# trans_velo.frame_id = 'odom'
	# _quat = tf.transformations.quaternion_from_euler(0,0,self.heading_rad)
	# trans_velo.transform.rotation.x = _quat[0]
	# trans_velo.transform.rotation.y = _quat[1]
	# trans_velo.transform.rotation.z = _quat[2]
	# trans_velo.transform.rotation.w = _quat[3]

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = 'odom'
	# cloud_out = do_transform_cloud(msg, trans_velo)
	if self.dist_accum >= self.dist_accum_last + self.dist_thres_for_adding_data or self.init_velo:
	    self.repub_velo.publish(msg)
	    self.dist_accum_last = self.dist_accum
	    self.init_velo = False
    def processImuMsg(self, msg):
	# print msg.angular_velocity.z
	self.heading_rad = self.heading_rad + msg.angular_velocity.z*.01
	# print self.heading_rad

	# quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
	# euler = tf.transformations.euler_from_quaternion(quat)
	# heading_from_imu = euler[2]

	# self.heading_rad = normalizeHeading(self.heading_rad)

	# heading_from_imu = normalizeHeading(euler[2])

	# print math.degrees(self.heading_rad) , math.degrees(euler[2])
	
	
	pass
    def processOdomMsg(self, msg):
	# print -msg.twist.twist.linear.x
	curr_odom = -msg.twist.twist.linear.x
	
	if self.init:
	    self.dist = (curr_odom - self.last_odom) * self.dist_to_meter
	    self.x_pose = self.x_pose - self.dist*math.sin(self.heading_rad)
	    self.y_pose = self.y_pose + self.dist*math.cos(self.heading_rad)
	    self.dist_accum = self.dist_accum + self.dist
	    # print self.x_pose, self.y_pose , self.dist_accum
	    self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), rospy.Time.now(), 'odom', "world")
	    # self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), self.velostamp, 'odom', "velodyne")
	    # self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), rospy.Time.now(), 'world', "odom")
	else:
	    self.pub_br.sendTransform((0 , 0 ,0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), 'odom', "world")


	self.last_odom = curr_odom
	self.init = True
	pass

g_dist_to_meter = 1./2700
g_dist_thres = .1#1#5#2
if __name__ == '__main__':
    odom = ImuEnco2Odom(g_dist_to_meter, g_dist_thres)
    odom.run()

