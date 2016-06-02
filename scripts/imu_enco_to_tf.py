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

# f_handle = open('odom_log_infuse.csv','w')
# f_handle = open('odom_log_infuse_1.csv','w')
# f_handle = open('odom_log_infuse_2.csv','w')

# f_handle_loc = open('odom_corrected.csv','w')


def savePose2D(x,y,yaw,file_handle):
  str_ = ''
  str_ = str_ + format(x,'.3f') + ' '
  str_ = str_ + format(y,'.3f') + ' '
  str_ = str_ + format(yaw,'.5f') + ' '
  str_ = str_ + '\n'
  file_handle.write(str_)

def publishOdomRos(x,y,yaw,publisher, header_frame, child_frame):
  odom_data = Odometry()
  odom_data.header.stamp = rospy.Time.now()
  odom_data.header.frame_id = header_frame
  odom_data.child_frame_id = child_frame
  odom_data.pose.pose.position.x = x
  odom_data.pose.pose.position.y = y
  _quat = tf.transformations.quaternion_from_euler(0,0,yaw)
  odom_data.pose.pose.orientation.x = _quat[0]
  odom_data.pose.pose.orientation.y = _quat[1]
  odom_data.pose.pose.orientation.z = _quat[2]
  odom_data.pose.pose.orientation.w = _quat[3]
  # print odom_data
  publisher.publish(odom_data)

def normalizeHeading(angle_rad):
    while angle_rad < 0:
	angle_rad = angle_rad + math.pi*2
    while angle_rad > 2*math.pi:
	angle_rad = angle_rad - math.pi*2
    return angle_rad

class ImuEnco2Odom:
    def __init__(self, dist_to_meter, dist_thres, odom_thres=.1):
	rospy.init_node('imu_enco_to_odom', anonymous=False)
	rospy.Subscriber('imu/data', Imu, self.processImuMsg)
	rospy.Subscriber('encoder_odom', Odometry, self.processOdomMsg)
	# rospy.Subscriber('velodyne_points', PointCloud2, self.processVeloMsg)
	self.repub_velo = rospy.Publisher('velotime_points', PointCloud2)
	self.pub_odom = rospy.Publisher("odometry", Odometry)
	self.pub_odom_corrected = rospy.Publisher("data_test_balaji", Odometry)
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

    def run(self):
	try:
	    rospy.spin()
	except rospy.ROSInterruptException:
	    # f_handle.close()
	    pass
	pass
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
	# msg.header.frame_id = 'odom'
	msg.header.frame_id = 'odom_corrected'
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
	# curr_odom = -msg.twist.twist.linear.x
	# for mobile base ONLY BACKWARD
	curr_odom = msg.twist.twist.linear.x
	
	str_ = ''
	if self.init:
	    self.dist = (curr_odom - self.last_odom) * self.dist_to_meter
	    # for miev with velo32
	    # self.x_pose = self.x_pose - self.dist*math.sin(self.heading_rad)
	    # self.y_pose = self.y_pose + self.dist*math.cos(self.heading_rad)
	    # for coms2
	    self.x_pose = self.x_pose + self.dist*math.cos(self.heading_rad)
	    self.y_pose = self.y_pose + self.dist*math.sin(self.heading_rad)

	    str_ = str_ + format(self.x_pose,'.3f') + ' '
	    str_ = str_ + format(self.y_pose,'.3f') + ' '
	    str_ = str_ + format(self.heading_rad,'.5f') + ' '
	    str_ = str_ + '\n'
	    # f_handle.write(str_)

	    # self.dist_accum = self.dist_accum + self.dist
	    self.dist_accum = self.dist_accum + abs(self.dist)
	    # print self.x_pose, self.y_pose , self.dist_accum
	    # if self.dist_accum >= self.dist_odom_accum_last + self.dist_thres_for_odom:
	    if True:
	      self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), rospy.Time.now(), 'odom', "world")
	      # self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), self.velostamp, 'odom', "velodyne")
	      # self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), rospy.Time.now(), 'world', "odom")
	      self.pub_br.sendTransform((self.x_pose , self.y_pose, 0), tf.transformations.quaternion_from_euler(0,0,self.heading_rad), rospy.Time.now(), 'odom_corrected', "lpm_correction")


	      odom_data = Odometry()
	      odom_data.header.stamp = rospy.Time.now()
	      odom_data.header.frame_id = 'world'
	      odom_data.child_frame_id = 'odom'
	      odom_data.pose.pose.position.x = self.x_pose
	      odom_data.pose.pose.position.y = self.y_pose
	      _quat = tf.transformations.quaternion_from_euler(0,0,self.heading_rad)
	      odom_data.pose.pose.orientation.x = _quat[0]
	      odom_data.pose.pose.orientation.y = _quat[1]
	      odom_data.pose.pose.orientation.z = _quat[2]
	      odom_data.pose.pose.orientation.w = _quat[3]
	      # print odom_data
	      self.pub_odom.publish(odom_data)
	      self.dist_odom_accum_last = self.dist_accum

	    try:
	      (trans_lpm,rot_lpm) = self.pub_lt.lookupTransform('/world', '/odom_corrected', rospy.Time(0))
	    except:
	      # tf lpm_correction not avail yet, waiting for first scan, nevermind, odom is enough.
	      (trans_lpm,rot_lpm) = self.pub_lt.lookupTransform('/world', '/odom', rospy.Time(0))

	    euler_lpm = tf.transformations.euler_from_quaternion(rot_lpm)
	    # self.pub_br.sendTransform(trans_lpm, rot_lpm, rospy.Time.now(), 'data_test_balaji', "world")
	    self.pub_br.sendTransform((trans_lpm[0],trans_lpm[1],0), tf.transformations.quaternion_from_euler(0,0,euler_lpm[2]), rospy.Time.now(), 'data_test_balaji', "world")
	    # savePose2D(trans_lpm[0],trans_lpm[1],euler_lpm[2],f_handle_loc)
	    publishOdomRos(trans_lpm[0],trans_lpm[1],euler_lpm[2],self.pub_odom_corrected,'world','data_test_balaji')
	    print euler_lpm

	else:
	    self.pub_br.sendTransform((0 , 0 ,0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), 'odom', "world")
	    # savePose2D(0,0,0,f_handle_loc)
	    publishOdomRos(0,0,0,self.pub_odom_corrected,'world','data_test_balaji')

	self.last_odom = curr_odom
	self.init = True
	pass


# g_dist_to_meter = 1./3000#1./2700 - for miev, 1./3000 for coms2
g_dist_to_meter = 1. #1./3000#1./2700 - for miev, 1./3000 for coms2
g_dist_thres = .5#2.5#.1#1#5#2
if __name__ == '__main__':
    odom = ImuEnco2Odom(g_dist_to_meter, g_dist_thres)
    odom.run()
    

