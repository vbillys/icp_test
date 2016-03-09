#!/usr/bin/env python
import socket
import struct
import sys
import os
import time
import binascii
import math
from random import randint
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf


#PC_IP = "192.168.0.145"    #127.0.0.1"
PC_IP = "192.168.0.102"    #127.0.0.1"
LIS_PORT = 64417

CONTROL_IP = "192.168.0.145"   #"127.0.0.1"
CONTROL_PORT = 41010


UDPSOCKS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP Send

def callbackProcessedPose(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	vl = 0
	vr = 0
	_fvalues = (int('F1', 16), int('A1', 16), x, y, yaw, vl, vr)
	_fstructure = struct.Struct('< ' + '2B d d d d d')
	_fdata = _fstructure.pack(*_fvalues)
	UDPSOCKS.sendto(_fdata, (CONTROL_IP, CONTROL_PORT))
	print "sending", x, y, yaw, vl, vr


def startRosNode(node_name):
	rospy.init_node(node_name, anonymous=False)
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callbackProcessedPose)
	rospy.spin()


if __name__ == '__main__':
	try:
		startRosNode('send_loc_to_control_node')
	except rospy.ROSInterruptException:
		pass
