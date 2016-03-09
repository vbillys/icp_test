#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
import sensor_msgs.point_cloud2 as pc2
import tf
import numpy
import math

rospy.init_node('point_cloud_combiner_with_2dposes_node', anonymous=False)

# g_point_cloud_built = PointCloud2()
g_point_cloud_built = [] 
g_points_new = []
g_pub_cloud = rospy.Publisher("ibeo_points_combined", PointCloud2)
g_pcloud = PointCloud2()
g_pose = PoseWithCovarianceStamped()

# def addPointsTo(pc1, pc2):
  # for point in pc2.points:


def processPointCloud(msg):
  global g_points_new
  # g_point_cloud_built.header = header;
  # g_point_cloud_built.points = g_point_cloud_built.points + msg.points
  points_new_gen = pc2.read_points(msg, skip_nans=True, field_names=("x","y","z"))
  g_points_new = [ p for p in points_new_gen]

  # g_pub_cloud.publish(pcloud)

def processPose2d(msg):
  global g_point_cloud_built
  global g_pcloud
  global g_pose
  g_pose = msg
  header = Header()
  header.stamp = rospy.Time.now()
  header.frame_id = 'ibeo'
  H_points = [[p[0], p[1],1] for p in g_points_new]
  M_points = numpy.matrix(H_points)
  # print M_points
  quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(quat)
  yaw = -euler[2]
  tranformation_matrix = numpy.matrix([[math.cos(yaw),math.sin(yaw),msg.pose.pose.position.x],[-math.sin(yaw),math.cos(yaw),msg.pose.pose.position.y],[0,0,1]])
  # tranformation_matrix = numpy.matrix([[math.cos(yaw),math.sin(yaw),0],[-math.sin(yaw),math.cos(yaw),0],[0,0,1]])
  # tranformation_matrix = numpy.matrix([[1,0,-msg.pose.pose.position.x],[0,1,-msg.pose.pose.position.y],[0,0,1]])

  point_t = tranformation_matrix *M_points.getT()
  point_t = point_t.getT().tolist()

  g_point_cloud_built = g_point_cloud_built + point_t
  # g_point_cloud_built =  point_t
  g_pcloud = pc2.create_cloud_xyz32(header, g_point_cloud_built)
  print msg.pose.pose.position.x, msg.pose.pose.position.y, yaw
  g_pub_cloud.publish(g_pcloud)
  pass

def talker():
  rospy.Subscriber('ibeo_points', PointCloud2, processPointCloud)
  rospy.Subscriber('mrpt_pose2d', PoseWithCovarianceStamped, processPose2d)
  rospy.spin()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass

