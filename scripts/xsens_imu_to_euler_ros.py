#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
import tf

def euler_to_degrees(input):
  output = [0,0,0]
  output[0] = round(math.degrees(input[0]),3)
  output[1] = round(math.degrees(input[1]),3)
  output[2] = round(math.degrees(input[2]),3)
  return output

def processImuMsg(msg):
  #print msg
  quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quat)
  euler = euler_to_degrees(euler)
  print euler
  

if __name__ == '__main__':
  rospy.init_node('enco_to_odom', anonymous=False)
  rospy.Subscriber('imu/data', Imu, processImuMsg)
  rospy.spin()


