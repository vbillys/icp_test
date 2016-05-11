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


class PoseToTF:
	def __init__(self, dist_to_meter, dist_thres, moving_odom_frame, map_frame, odom_topic_name, odom_thres=.1):
		rospy.init_node('pose_to_tf', anonymous=False)
		rospy.Subscriber(odom_topic_name, Odometry, self.processOdomMsg)
		# rospy.Subscriber('RosAria/pose', Odometry, self.processOdomMsg)
		# rospy.Subscriber('odom', Odometry, self.processOdomMsg)
		rospy.Subscriber('velodyne_points', PointCloud2, self.processVeloMsg)
		self.repub_velo = rospy.Publisher('velotime_points', PointCloud2)
		self.dist_to_meter = dist_to_meter
		self.dist_thres_for_adding_data = dist_thres
		self.dist_thres_for_odom = odom_thres
		self.heading_rad = 0
		self.x_pose = 0
		self.y_pose = 0
		self.dist   = 0
		self.dist_accum   = 0
		self.dist_accum_last = 0
		self.dist_odom_accum_last = 0
		self.last_odom = 0
		self.init = False
		self.init_velo = True
		self.pub_br = tf.TransformBroadcaster()
		self.pub_lt = tf.TransformListener()
		self.last_odom_msg = Odometry()
		self.moving_odom_frame = moving_odom_frame
		self.map_frame = map_frame
		self.odom_topic_name = odom_topic_name

	def run(self):
		try:
			rospy.spin()
		except rospy.ROSInterruptException:
			pass
			# f_handle.close()
		pass


	def processVeloMsg(self, msg):
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'odom_corrected'
		if self.dist_accum >= self.dist_accum_last + self.dist_thres_for_adding_data or self.init_velo:
			self.repub_velo.publish(msg)
			self.dist_accum_last = self.dist_accum
			self.init_velo = False
	def processOdomMsg(self, msg):
		if self.init:
			x_diff = msg.pose.pose.position.x - self.last_odom_msg.pose.pose.position.x
			y_diff = msg.pose.pose.position.y - self.last_odom_msg.pose.pose.position.y

			first_quat = (self.first_odom_msg.pose.pose.orientation.x, self.first_odom_msg.pose.pose.orientation.y, self.first_odom_msg.pose.pose.orientation.z, self.first_odom_msg.pose.pose.orientation.w)
			first_euler = tf.transformations.euler_from_quaternion(first_quat)
			quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)

			self.dist = math.hypot( x_diff
					      , y_diff)
			self.dist_accum = self.dist_accum + self.dist

			self.heading_rad = euler[2] - first_euler[2]
			self.x_pose = msg.pose.pose.position.x - self.first_odom_msg.pose.pose.position.x
			self.y_pose = msg.pose.pose.position.y - self.first_odom_msg.pose.pose.position.y
			self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), rospy.Time.now(), self.moving_odom_frame, self.map_frame)
			pass
		else:
			self.pub_br.sendTransform((0 , 0 ,0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), self.moving_odom_frame, self.map_frame)
			self.first_odom_msg = msg
			pass
		# self.last_odom = curr_odom
		self.last_odom_msg = msg
		self.init = True

g_dist_to_meter = 1. #1./3000#1./2700 - for miev, 1./3000 for coms2
g_dist_thres = .5#2.5#.1#1#5#2
# g_moving_odom_frame = 'base_footprint'
g_moving_odom_frame = 'odom'
# g_map_frame = 'nav'
g_map_frame = 'world'
# g_odometry_topic = 'odom'
g_odometry_topic = 'RosAria/pose'
if __name__ == '__main__':
	odom = PoseToTF(g_dist_to_meter, g_dist_thres, g_moving_odom_frame, g_map_frame, g_odometry_topic)
	# print odom.last_odom
	odom.run()


