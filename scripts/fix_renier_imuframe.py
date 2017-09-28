#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import Imu
import tf

g_pub_imu = []
g_first_time = True
g_prev_msg_time = 0

def euler_to_degrees(input):
  output = [0,0,0]
  output[0] = round(math.degrees(input[0]),3)
  output[1] = round(math.degrees(input[1]),3)
  output[2] = round(math.degrees(input[2]),3)
  return output

def processImuMsg(msg):
    #print msg
  global g_first_time
  global g_prev_msg_time
  if g_first_time:
      g_prev_msg_time = msg.header.stamp
      g_first_time = False
      return
  quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
  # R =  tf.transformations.quaternion_matrix(quat)
  # # R[0:3, 2] = - R[0:3, 2]
  # R[2, 0:3] = - R[2, 0:3]
  # roll, pitch, yaw = tf.transformations.euler_from_matrix(R)
  # re_quat = tf.transformations.quaternion_from_matrix(R)
  # euler = tf.transformations.euler_from_quaternion(quat)
  # euler = euler_to_degrees(euler)
  # print euler
  # # print R
  # print euler_to_degrees([roll, pitch, yaw])
  remsg = Imu()
  remsg.header.frame_id = 'imu_renier'
  # remsg.header.stamp = msg.header.stamp
  remsg.header.stamp = rospy.Time.now()
  remsg.orientation = quat
  remsg.angular_velocity = msg.angular_velocity
  remsg.linear_acceleration = msg.linear_acceleration
  # remsg.orientation.x = re_quat[0]
  # remsg.orientation.y = re_quat[1]
  # remsg.orientation.z = re_quat[2]
  # remsg.orientation.w = re_quat[3]
  remsg.linear_acceleration = msg.linear_acceleration;
  msg.header.frame_id = 'imu_renier'
  timestamp_diff = msg.header.stamp - g_prev_msg_time
  if not math.isnan(msg.linear_acceleration.z) and timestamp_diff > rospy.Duration(0):
      g_prev_msg_time = msg.header.stamp
      g_pub_imu.publish(msg)


if __name__ == '__main__':
  rospy.init_node('fix_renier_imuframe', anonymous=False)
  rospy.Subscriber('an_device/Imu_raw', Imu, processImuMsg)
  g_pub_imu = rospy.Publisher('imu/data', Imu)
  rospy.spin()


