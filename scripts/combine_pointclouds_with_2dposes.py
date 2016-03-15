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
g_pub_cloud_only_current = rospy.Publisher("ibeo_points_icp_only_current", PointCloud2)
g_pcloud = PointCloud2()
g_pose = PoseWithCovarianceStamped()
g_dist_accum = 0
g_dist_thres = 10.
g_last_dist_accum = - g_dist_thres
g_last_x = 0 
g_last_y = 0
g_last_captured_x = 0
g_last_captured_y = 0
g_last_captured_yaw = 0

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
  global g_dist_accum
  global g_last_dist_accum
  global g_last_x
  global g_last_y
  global g_last_captured_x
  global g_last_captured_y
  global g_last_captured_yaw
  g_pose = msg
  quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(quat)
  yaw = -euler[2]
  g_dist_accum = g_dist_accum + math.hypot(msg.pose.pose.position.x - g_last_x, msg.pose.pose.position.y - g_last_y)
  g_last_x = msg.pose.pose.position.x
  g_last_y = msg.pose.pose.position.y

  if g_dist_accum > g_last_dist_accum + g_dist_thres:
    g_last_dist_accum = g_last_dist_accum + g_dist_thres
    tranformation_matrix = numpy.matrix([[math.cos(yaw),math.sin(yaw),msg.pose.pose.position.x],[-math.sin(yaw),math.cos(yaw),msg.pose.pose.position.y],[0,0,1]])
    # tranformation_matrix = numpy.matrix([[math.cos(yaw),math.sin(yaw),0],[-math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    # tranformation_matrix = numpy.matrix([[1,0,-msg.pose.pose.position.x],[0,1,-msg.pose.pose.position.y],[0,0,1]])

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'ibeo'
    H_points = [[p[0], p[1],1] for p in g_points_new]
    M_points = numpy.matrix(H_points)
    # print M_points
    point_t = tranformation_matrix *M_points.getT()
    point_t = point_t.getT().tolist()
    g_point_cloud_built = g_point_cloud_built + point_t
    # g_point_cloud_built =  point_t
    g_pcloud = pc2.create_cloud_xyz32(header, g_point_cloud_built)
    pc_point_t = pc2.create_cloud_xyz32(header, point_t)
    observed_x = -g_last_captured_x + msg.pose.pose.position.x
    observed_y = -g_last_captured_y + msg.pose.pose.position.y
    observed_x_local =  math.cos(-yaw)*observed_x + math.sin(-yaw)*observed_y
    observed_y_local = -math.sin(-yaw)*observed_x + math.cos(-yaw)*observed_y
    print msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, observed_x_local,observed_y_local,-g_last_captured_yaw +yaw, msg.pose.covariance[0], msg.pose.covariance[1],msg.pose.covariance[2],msg.pose.covariance[3],msg.pose.covariance[4],   msg.pose.covariance[5]
    g_pub_cloud.publish(g_pcloud)
    g_pub_cloud_only_current.publish(pc_point_t)
    g_last_captured_x = msg.pose.pose.position.x
    g_last_captured_y = msg.pose.pose.position.y
    g_last_captured_yaw = yaw
  pass

def talker():
  # rospy.Subscriber('ibeo_points', PointCloud2, processPointCloud)
  rospy.Subscriber('ibeo_points_filtered', PointCloud2, processPointCloud)
  rospy.Subscriber('mrpt_pose2d', PoseWithCovarianceStamped, processPose2d)
  rospy.spin()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass

