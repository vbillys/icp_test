#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from roslib import message
import tf

import math, numpy
import sys

import xyz
import re
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import transformation

rospy.init_node('point_cloud_repub', anonymous=False)
pub_cloud = rospy.Publisher("filtered_points", PointCloud2)
pub_odom = rospy.Publisher("odometry", Odometry)
pub_frame = rospy.Publisher("frame", Odometry)
pub_br = tf.TransformBroadcaster()



from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np

from pyrr import Quaternion, Matrix33, Matrix44, Vector4
import pyrr

app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.opts['distance'] = 20
w.show()
w.setWindowTitle('pyqtgraph example: GLScatterPlotItem')

g = gl.GLGridItem()
w.addItem(g)


# ##
# ##  First example is a set of points with pxMode=False
# ##  These demonstrate the ability to have points with real size down to a very small scale 
# ## 
# pos = np.empty((53, 3))
# size = np.empty((53))
# color = np.empty((53, 4))
# pos[0] = (1,0,0); size[0] = 0.5;   color[0] = (1.0, 0.0, 0.0, 0.5)
# pos[1] = (0,1,0); size[1] = 0.2;   color[1] = (0.0, 0.0, 1.0, 0.5)
# pos[2] = (0,0,1); size[2] = 2./3.; color[2] = (0.0, 1.0, 0.0, 0.5)

# z = 0.5
# d = 6.0
# for i in range(3,53):
    # pos[i] = (0,0,z)
    # size[i] = 2./d
    # color[i] = (0.0, 1.0, 0.0, 0.5)
    # z *= 0.5
    # d *= 2.0

# sp1 = gl.GLScatterPlotItem(pos=pos, size=size, color=color, pxMode=False)
# sp1.translate(5,5,0)
# w.addItem(sp1)


def filter_points(msg):
    print 'got point cloud...'
    #pcloud = PointCloud2()
    #pub_cloud.publish(pcloud)
    #pub_cloud.publish(msg)
    print msg.header
    print msg.height
    print msg.width
    print msg.fields
    print msg.is_bigendian
    print msg.point_step
    print msg.row_step
    print msg.is_dense

# g_data_dir = '/home/avavav/Documents/workspace/3dtk/hannover1/'
# g_data_dir = '/home/avavav/Documents/workspace/3dtk/jtc1/'
# g_data_dir = '/home/avavav/Documents/workspace/3dtk/jtc2/'
# g_data_dir = '/home/avavav/Documents/workspace/3dtk/jtc3/'

# g_data_dir = '/home/avavav/Documents/workspace/3dtk/jtcpark1/'

# g_data_dir = '/home/avavav/Documents/workspace/3dtk/jtcpark2/'
# g_data_dir = '/home/avavav/Documents/workspace/3dtk/infuse/'
# g_data_dir = '/home/avavav/Documents/workspace/3dtk/infuse2/'
# g_data_dir = '/home/avavav/Documents/workspace/3dtk/infuse1/'


# g_data_dir = '/home/avavav/Documents/workspace/3dtk/connexis_set1/'
g_data_dir = '/home/avavav/Documents/workspace/3dtk/skygarden_set1/'

g_scale_factor = 0.01
g_trans_matrix = transformation.identity_matrix()

def extract_pose(content_pose):
    # pose_trans_parsed = re.findall( r'\d+\.*\d*', content_pose[0])
    pose_trans_parsed = content_pose[0].split(' ')
    # pose_rot_parsed   = re.findall( r'\d+\.*\d*', content_pose[1])
    pose_rot_parsed = content_pose[1].split(' ')
    # print pose_rot_parsed, pose_trans_parsed
    heading_deg = -float(pose_rot_parsed[1])
    heading_rad = heading_deg*math.pi/180
    x_pose = float(pose_trans_parsed[0])*g_scale_factor
    y_pose = float(pose_trans_parsed[2])*g_scale_factor
    # x_pose = float(pose_trans_parsed[0])*g_scale_factor*math.sin(heading_rad)
    # y_pose = -float(pose_trans_parsed[2])*g_scale_factor*math.cos(heading_rad)
    print x_pose, y_pose, heading_deg
    Rz = transformation.rotation_matrix(heading_rad, [0,0,1])
    return Rz, x_pose, y_pose, heading_rad 

def extractCorrPose(content_pose):
    t_parsed = content_pose[-1].split(' ')
    t_parsed = t_parsed[0:16]
    # print t_parsed
    # t_parsed = re.findall( r'\d+\.*\d*', content_pose[-1])
    t_parsed_floated = [ float(m) for m in t_parsed ]
    print t_parsed_floated, t_parsed, content_pose[-1]
    t_parsed_numpy =  numpy.matrix(t_parsed_floated[0:16])
    # return numpy.reshape(t_parsed_numpy, (4, 4))
    return t_parsed_numpy.reshape((4,4)).getT()
    # return t_parsed_numpy.reshape((4,4))
    

def fakeReadlines(cnt):
    idata = [
	    [ '1 0 0 0 0 1 0 0 0 0 -1 0 0 0 0 1 '],
   [ '0.999926 0.000147967 0.0121805 0 -0.000145486 1 -0.000204573 0 0.0121805 -0.000202786 -0.999926 0 0.666111 -0.389276 0.0564839 1 '],
   [ '0.999993 -0.00150819 0.00346573 0 0.00149107 0.999987 0.00493766 0 0.00347313 0.00493246 -0.999982 0 4.17811 1.81719 -24.7345 1 '],
   [ '0.62136 -0.0114647 -0.783441 0 0.0074911 0.999934 -0.00869154 0 -0.783489 0.000468253 -0.621405 0 -28.0668 -2.18228 -125.379 1 '],
   [ '0.613764 -0.0232933 -0.789146 0 0.0128312 0.999727 -0.0195295 0 -0.789385 -0.00186083 -0.613896 0 -142.454 -7.87962 -217.798 1 '],
   [ '0.623287 -0.0130675 -0.781884 0 0.0114808 0.999906 -0.00755925 0 -0.781909 0.00426509 -0.623378 0 -279.586 1.1309 -323.555 1 '],
   [ '0.622152 -0.0146684 -0.782759 0 0.00889735 0.999892 -0.0116656 0 -0.782846 -0.000293308 -0.622216 0 -417.597 -1.77557 -428.382 1 '],
   [ '0.636223 -0.0120064 -0.771411 0 0.00865927 0.999927 -0.00842137 0 -0.771456 0.00132199 -0.636281 0 -542.427 -9.30168 -522.356 1 '],
   [ '0.994442 -0.0119872 0.104604 0 0.0134851 0.999816 -0.0136239 0 0.104422 -0.0149588 -0.994421 0 -595.113 -12.6048 -627.678 1 '],
   [ '0.994922 -0.0444333 0.090305 0 0.0453884 0.998933 -0.00854942 0 0.0898288 -0.0126048 -0.995877 0 -584.834 -18.7123 -742.517 1 '],
   [ '0.885376 -0.0335246 -0.463665 0 0.0336297 0.999402 -0.00804369 0 -0.463657 0.00847123 -0.885974 0 -601.423 -18.5951 -858.439 1 '],
   [ '0.22826 -0.010666 -0.973542 0 0.0196903 0.999786 -0.00633684 0 -0.973401 0.0177229 -0.228421 0 -688.219 -18.6678 -941.187 1 '],
   [ '0.223829 -0.0183521 -0.974456 0 0.0164097 0.999752 -0.0150593 0 -0.97449 0.0126198 -0.224075 0 -835.163 -12.9162 -979.666 1 '],
   [ '0.24031 -0.022661 -0.970432 0 -0.00645353 0.999668 -0.0249418 0 -0.970675 -0.0122565 -0.240084 0 -988.303 -15.8202 -1009.39 1 '],
   [ '0.234768 -0.0262747 -0.971696 0 -0.0124697 0.999471 -0.0300384 0 -0.971972 -0.0191688 -0.234316 0 -1169.87 -21.5382 -1056.05 1 '],
   [ '0.245878 -0.0211575 -0.96907 0 -0.0202923 0.99943 -0.026969 0 -0.969088 -0.0262958 -0.245309 0 -1387.33 -30.069 -1111.72 1 ']
    ]
    return idata[cnt-1]


def read_xyz(fh, fh_frame, fh_pose, cnt, force_2d = False):
    global g_trans_matrix
    content_frame = fh_frame.readlines()
    tranformation_matrix = extractCorrPose(content_frame)
    print tranformation_matrix
    # content_frame = fakeReadlines(cnt)
    content_pose  = fh_pose.readlines()
    Rz, x_pose, y_pose, heading_rad = extract_pose(content_pose)
    # tranformation_matrix = extractCorrPose(content_frame)


    

    content = fh.readlines()
    cloud = []
    cloud_raw = []
    for s in content:
	# s_parsed = re.findall( r'\d+\.*\d*', s)
	s_parsed = s.split(' ')
	# s_parsed_floated = []
	# for _f in s_parsed:
	    # s_parsed_floated.append(float(_f)*.1)

	# cloud.append(s_parsed_floated)

	# point  = [float(s_parsed[2])*g_scale_factor,-float(s_parsed[0])*g_scale_factor,float(s_parsed[1])*g_scale_factor]
	# point  = [-float(s_parsed[0])*g_scale_factor,float(s_parsed[2])*g_scale_factor,float(s_parsed[1])*g_scale_factor]
	point  = [float(s_parsed[0]),float(s_parsed[1]),float(s_parsed[2])]
	# point_t = Rz.dot([point[0], point[1], point[2], 0])
	# print point_t
	# cloud.append(point)
	# cloud.append([point_t[0]+x_pose, point_t[1]+y_pose, point_t[2]])
	# cloud.append([point_t[0], point_t[1], point_t[2]])

	# cloud.append([point[0], point[1], point[2], 1])
	cloud.append([point[0]*g_scale_factor, point[2]*g_scale_factor, point[1]*g_scale_factor, 1])
	cloud_raw.append([point[0]*g_scale_factor, point[2]*g_scale_factor, point[1]*g_scale_factor])

    # point_t = tranformation_matrix.dot([point[0], point[1], point[2], 0])
    cloud_matrix = numpy.matrix(cloud)#.reshape((len(content),4))
    # g_trans_matrix = numpy.dot(g_trans_matrix, tranformation_matrix )
    # g_trans_matrix = numpy.dot(tranformation_matrix, g_trans_matrix )
    # point_t = g_trans_matrix.dot(cloud_matrix.getT())
    # point_t = tranformation_matrix.dot(cloud_matrix.getT())

    mirror = numpy.matrix([[1,0,0,0],[0,1, 0,0],[0,0,-1,0],[0,0,0,1]])
    mirror2= numpy.matrix([[1,0,0,0],[0,0,-1,0],[0,1, 0,0],[0,0,0,1]])
    mirror3= numpy.matrix([[1,0,0,0],[0,0, 1,0],[0,-1,0,0],[0,0,0,1]])
    mirror4= numpy.matrix([[1,0,0,0],[0,-1, 0,0],[0,0,1,0],[0,0,0,1]])

    if force_2d:
	tranformation_matrix[2,3] = 0
	al, be, ga = tf.transformations.euler_from_matrix(tt_tran, 'rxyz')
	new_rot_mat = tf.transformations.euler_matrix(0,0,ga, 'rxyz')
	tranformation_matrix[0:3,0:3] = new_rot_mat[0:3,0:3]

    # tranformation_matrix = mirror2 * mirror * tranformation_matrix
    # tranformation_matrix =  tranformation_matrix* mirror


    t_temptrans = tranformation_matrix
    # tt_tran = numpy.matrix(tranformation_matrix)
    tt_tran = numpy.empty_like(tranformation_matrix)
    tt_tran[:] = tranformation_matrix
    tt_tran [0,1] = -t_temptrans.item((0,2))
    tt_tran [1,1] = -t_temptrans.item((1,2))
    tt_tran [2,1] = t_temptrans.item((2,2))
    tt_tran [0,0] = t_temptrans.item((0,0))
    tt_tran [1,0] = t_temptrans.item((1,0))
    tt_tran [2,0] = -t_temptrans.item((2,0))
    tt_tran [0,2] = t_temptrans.item((0,1))
    tt_tran [1,2] = t_temptrans.item((1,1))
    tt_tran [2,2] = -t_temptrans.item((2,1))
    tranformation_matrix[0,3] = tranformation_matrix[0,3]*g_scale_factor
    tranformation_matrix[1,3] = tranformation_matrix[1,3]*g_scale_factor
    tranformation_matrix[2,3] = tranformation_matrix[2,3]*g_scale_factor


    tranformation_matrix =  mirror2 * mirror * tranformation_matrix * mirror3 * mirror4
    # tranformation_matrix[0:3,0:3] = tt_tran[0:3,0:3]

    print tranformation_matrix
    euler_from_trans_mat =  tf.transformations.euler_from_matrix(tranformation_matrix, axes='sxyz')
    print euler_from_trans_mat
    quat1 = tf.transformations.quaternion_from_matrix(tranformation_matrix)
    print quat1
    quat2 =  tf.transformations.quaternion_from_euler(euler_from_trans_mat[0],euler_from_trans_mat[1],euler_from_trans_mat[2], axes='sxyz')
    print quat2
    print Matrix44(Matrix44(tranformation_matrix).quaternion)
    print Matrix44(pyrr.quaternion.create(x=quat1[0],y=quat1[1],z=quat1[2],w=quat1[3]))
    transformation_matrix =  Matrix44(pyrr.quaternion.create(x=quat2[0],y=quat2[1],z=quat2[2],w=quat2[3]))
    transformation_matrix[0,3] = tranformation_matrix[0,3]
    transformation_matrix[1,3] = tranformation_matrix[1,3]
    transformation_matrix[2,3] = tranformation_matrix[2,3]
    # tranformation_matrix = transformation_matrix


    # pub_br.sendTransform((tranformation_matrix.item((0,3))*g_scale_factor, tranformation_matrix.item((2,3))*g_scale_factor, tranformation_matrix.item((1,3))*g_scale_factor), tf.transformations.quaternion_from_matrix(tt_tran), rospy.Time.now(), 'frame', "velodyne")
    # pub_br.sendTransform((tranformation_matrix.item((0,3))*g_scale_factor, tranformation_matrix.item((1,3))*g_scale_factor, tranformation_matrix.item((2,3))*g_scale_factor), tf.transformations.quaternion_from_matrix(tt_tran), rospy.Time.now(), 'frame', "velodyne")
    # pub_br.sendTransform((tranformation_matrix.item((0,3)), tranformation_matrix.item((1,3)), tranformation_matrix.item((2,3))), tf.transformations.quaternion_from_matrix(tt_tran), rospy.Time.now(), 'frame', "velodyne")
    pub_br.sendTransform((tranformation_matrix.item((0,3)), tranformation_matrix.item((1,3)), tranformation_matrix.item((2,3))), tf.transformations.quaternion_from_matrix(tranformation_matrix), rospy.Time.now(), 'frame', "velodyne")
    # pub_br.sendTransform((tranformation_matrix.item((0,3)), tranformation_matrix.item((1,3)), tranformation_matrix.item((2,3))), tf.transformations.quaternion_from_euler(euler_from_trans_mat[0],euler_from_trans_mat[1],euler_from_trans_mat[2], axes='sxyz'), rospy.Time.now(), 'frame', "velodyne")
    # pub_br.sendTransform((tranformation_matrix.item((0,3)), tranformation_matrix.item((1,3)), tranformation_matrix.item((2,3))), Matrix44(tranformation_matrix).quaternion, rospy.Time.now(), 'frame', "velodyne")
    pub_br.sendTransform((x_pose , y_pose, 0), tf.transformations.quaternion_from_euler(0,0,heading_rad), rospy.Time.now(), 'odom', "velodyne")

    odom_data = Odometry()
    odom_data.header.stamp = rospy.Time.now()
    odom_data.header.frame_id = 'velodyne'
    odom_data.child_frame_id = 'odom'
    odom_data.pose.pose.position.x = x_pose
    odom_data.pose.pose.position.y = y_pose
    _quat = tf.transformations.quaternion_from_euler(0,0,heading_rad)
    odom_data.pose.pose.orientation.x = _quat[0]
    odom_data.pose.pose.orientation.y = _quat[1]
    odom_data.pose.pose.orientation.z = _quat[2]
    odom_data.pose.pose.orientation.w = _quat[3]
    # print odom_data
    pub_odom.publish(odom_data)

    frame_data = Odometry()
    frame_data.header.stamp = rospy.Time.now()
    frame_data.header.frame_id = 'velodyne'
    frame_data.child_frame_id = 'frame'
    # frame_data.pose.pose.position.x = tranformation_matrix.item((0,3))*g_scale_factor
    # frame_data.pose.pose.position.y = tranformation_matrix.item((2,3))*g_scale_factor
    # frame_data.pose.pose.position.z = tranformation_matrix.item((1,3))*g_scale_factor


    # frame_data.pose.pose.position.x = tranformation_matrix.item((0,3))*g_scale_factor
    # frame_data.pose.pose.position.y = tranformation_matrix.item((1,3))*g_scale_factor
    # frame_data.pose.pose.position.z = tranformation_matrix.item((2,3))*g_scale_factor

    frame_data.pose.pose.position.x = tranformation_matrix.item((0,3))
    frame_data.pose.pose.position.y = tranformation_matrix.item((1,3))
    frame_data.pose.pose.position.z = tranformation_matrix.item((2,3))

    # frame_data.pose.pose.position.x = tranformation_matrix.item((0,2))
    # frame_data.pose.pose.position.y = tranformation_matrix.item((1,2))
    # frame_data.pose.pose.position.z = tranformation_matrix.item((2,2))
    # _quat = tf.transformations.quaternion_from_euler(0,0,heading_rad)


    # _quat = tf.transformations.quaternion_from_matrix(tt_tran)

    _quat = tf.transformations.quaternion_from_matrix(tranformation_matrix)

    # _quat = tf.transformations.quaternion_from_euler(euler_from_trans_mat[0],euler_from_trans_mat[1],euler_from_trans_mat[2])
    frame_data.pose.pose.orientation.x = _quat[0]
    frame_data.pose.pose.orientation.y = _quat[1]
    frame_data.pose.pose.orientation.z = _quat[2]
    frame_data.pose.pose.orientation.w = _quat[3]
    # print frame_data
    pub_frame.publish(frame_data)

    # tranformation_matrix =  mirror2 * mirror * tranformation_matrix * mirror3
    # tranformation_matrix =  mirror2 * tranformation_matrix 
    # tranformation_matrix =   tranformation_matrix * mirror4

    point_t = tranformation_matrix *cloud_matrix.getT()
    # print point_t
    point_t = point_t.getT().tolist()
    # point_t=point_t[0]

    cloud_out = []
    for row in point_t:
	# cloud_out.append([row[0]*g_scale_factor, -row[2]*g_scale_factor, row[1]*g_scale_factor])
	# cloud_out.append([row[0]*g_scale_factor, row[1]*g_scale_factor, row[2]*g_scale_factor])
	cloud_out.append([row[0], row[1], row[2]])

	# print s, s_parsed, s_parsed_floated

    # return s_parsed_floated
    # return cloud
    # print cloud_out

    return cloud_out
    # return cloud_matrix.tolist()
    # return cloud_raw

def publish_xyz (filename, cnt, force_2d=False):
    fullfilename_cloud = g_data_dir + filename + '.3d'
    fullfilename_pose  = g_data_dir + filename + '.pose'
    fullfilename_frame = g_data_dir + filename + '.frames'
    fh = open(fullfilename_cloud, 'r')
    fh_pose  = open(fullfilename_pose, 'r')
    fh_frame = open(fullfilename_frame, 'r')
    cloud = read_xyz(fh, fh_frame, fh_pose, cnt, force_2d = force_2d)
    # pcloud = PointCloud2()
    # header = Header()
    # header.stamp = rospy.Time.now()
    # header.frame_id = 'velodyne'
    
    # pcloud = pc2.create_cloud_xyz32(header, cloud)
    # pub_cloud.publish(pcloud)
    # print pcloud
    # print cloud
    return cloud

def saveCloud(filename, cloud):
	fh = open(filename, 'w')
	for (x,y,z) in cloud:
		str_ = format(x, '.3f')    + ' ' + format(y, '.3f')     + ' ' + format(z, '.3f')     + '\n'
		fh.write(str_)
	fh.close()



NO_LAST_FRAME = 136 #62#100#101#43
NO_START_FRAME = 1#61#43
NO_ROS_PUBLISHING = False #True
RATE_ROS_PUBLISHING = 3
FORCE_2D = False #True
def talker():

    rate = rospy.Rate(RATE_ROS_PUBLISHING)
    counter_index = NO_START_FRAME #1#21#30#x1
    pcloud = PointCloud2()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'velodyne'
    cloud = []
    while not rospy.is_shutdown():

	file_string = 'scan'+format(counter_index,'03d')
	cloud_new = publish_xyz (file_string, counter_index, force_2d = FORCE_2D)
	# cloud = cloud + cloud_new
	cloud =  cloud_new
	print file_string
	counter_index = counter_index + 1

	rate.sleep()
	if not NO_ROS_PUBLISHING:
		pcloud = PointCloud2()
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'velodyne' #'odom' #'pose' #'frame' #'velodyne'
		# header.frame_id = 'frame' #'odom' #'pose' #'frame' #'velodyne'
		# header.frame_id = 'odom' #'odom' #'pose' #'frame' #'velodyne'
		pcloud = pc2.create_cloud_xyz32(header, cloud)
		pub_cloud.publish(pcloud)

	if counter_index > NO_LAST_FRAME: #47:#91:#65:#19 :#65: #65: # 20 :#65 :# 468:
	    break
	    counter_index = 1
	rate.sleep()

    saveCloud("points.pts",cloud)

    # pcloud = pc2.create_cloud_xyz32(header, cloud)
    # print pcloud.header, pcloud.height, pcloud.width, pcloud.point_step, pcloud.row_step, pcloud.fields
    # pub_cloud.publish(pcloud)

    # rospy.spin()
    
    # pos = np.random.random(size=(100,3))
    # print cloud
    pos = np.matrix(cloud)
    # pos *= [10,-10,10]
    # pos[0] = (0,0,0)
    # color = np.ones((pos.shape[0], 4))
    # d2 = (pos**2).sum(axis=1)**0.5
    # size = np.random.random(size=pos.shape[0])*10
    size = np.ones((1,pos.shape[0]))
    sp2 = gl.GLScatterPlotItem(pos=pos, color=(1,1,1,1), size=size)
    w.addItem(sp2)

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
	QtGui.QApplication.instance().exec_()


rospy.Subscriber('velodyne_points', PointCloud2, filter_points)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
