import os
import numpy as np
import math
from ZipLib import ZipInputStream
import string, StringIO
import pylab as plt
import matplotlib.animation as animation
from cython_catkin_example import cython_catkin_example
import collections

import pcl
import ujson
import zlib

def createToroEdgeString(observed_vertex_id ,observing_vertex_id ,forward ,sideward ,rotate ,inf_ff ,inf_fs ,inf_ss ,inf_rr ,inf_fr ,inf_sr ):
	str_edge_added = ''
	str_edge_added = str_edge_added + 'EDGE2 '
	str_edge_added = str_edge_added + format(observed_vertex_id ,'d') + ' '
	str_edge_added = str_edge_added + format(observing_vertex_id,'d') + ' '
	str_edge_added = str_edge_added + format(forward ,'.4f') + ' '
	str_edge_added = str_edge_added + format(sideward ,'.4f') + ' '
	str_edge_added = str_edge_added + format(rotate ,'.4f') + ' '
	str_edge_added = str_edge_added + format(inf_ff ,'.4f') + ' '
	str_edge_added = str_edge_added + format(inf_fs ,'.4f') + ' '
	str_edge_added = str_edge_added + format(inf_ss ,'.4f') + ' '
	str_edge_added = str_edge_added + format(inf_rr ,'.4f') + ' '
	str_edge_added = str_edge_added + format(inf_fr ,'.4f') + ' '
	str_edge_added = str_edge_added + format(inf_sr ,'.4f') + ' '
	str_edge_added = str_edge_added + '\n'
	return str_edge_added

def createToroVertexString(vertex_id, x, y, yaw):
	str_vertex = ''
	str_vertex = str_vertex + 'VERTEX2 '
	str_vertex = str_vertex + format(vertex_id,'d') + ' '
	str_vertex = str_vertex + format(x,'.4f') + ' '
	str_vertex = str_vertex + format(y,'.4f') + ' '
	str_vertex = str_vertex + format(yaw,'.4f')
	str_vertex = str_vertex + '\n'
	return str_vertex

def getVertexFromGraphAutoCount(f_handle,  processed=False):
	f_content = f_handle.readlines()
	points = []
	# for str_ in f_content:
	for i in range(0,len(f_content)):
		strs = f_content[i].strip()
		strs_splitted =  strs.split()
		# print strs_splitted
		if strs_splitted[0] == "VERTEX" or strs_splitted[0] == "VERTEX2":
			noid = int(strs_splitted[1])
			if processed:
				coord = [-float(strs_splitted[2]), float(strs_splitted[3]), -float(strs_splitted[4])]
			else:
				coord = [float(strs_splitted[2]), float(strs_splitted[3]), float(strs_splitted[4])]
			points.append([noid]+coord)
	return points

def getEdgeFromGraphAutoCount(f_handle,  processed=False, override_covariances=False):
	f_content = f_handle.readlines()
	points_dict = {} #[]
	# for str_ in f_content:
	for i in range(0,len(f_content)):
		strs = f_content[i].strip()
		strs_splitted =  strs.split()
		# print strs_splitted
		if strs_splitted[0] == "EDGE" or strs_splitted[0] == "EDGE2":
			if processed:
				observed_id = int(strs_splitted[2])
				observing_id = int(strs_splitted[1])
			else:
				observed_id = int(strs_splitted[1])
				observing_id = int(strs_splitted[2])


			if processed:
				if override_covariances:
					coord = [-float(strs_splitted[3]), float(strs_splitted[4]), -float(strs_splitted[5])] + [1., 0., 1., 1., 0., 0.]
				else:
					coord = [-float(strs_splitted[3]), float(strs_splitted[4]), -float(strs_splitted[5])] + map(float, strs_splitted[6:])
			else:
				if override_covariances:
					coord = [float(strs_splitted[3]), float(strs_splitted[4]), float(strs_splitted[5])] + [1., 0., 1., 1., 0., 0.]
				else:
					coord = [float(strs_splitted[3]), float(strs_splitted[4]), float(strs_splitted[5])] + map(float, strs_splitted[6:])
			# points.append([observed_id, observing_id]+coord)
			points_dict[(observing_id, observed_id)] = coord
	return points_dict

def getFloatNumberFromReadLines(f_handle, no_params):
	f_content = f_handle.readlines()
	points = []
	for str_ in f_content:
		strs = str_.strip()
		point_ = map(float, strs.split())
		point = zip(*(iter(point_),) * no_params)
		points.append(point[0])
	# print points
	return points

def getFloatNumberListFromReadLines(f_handle):
	f_content = f_handle.readlines()
	points = []
	for str_ in f_content:
		strs = str_.strip()
		point_ = map(float, strs.split())
		# point = zip(*(iter(point_),) * no_params)
		points.append(point_)
	# print points
	return points

def getFloatNumberFromStringReadLines(strs, no_params):
	points = []
	for str_ in strs:
		strs = str_.strip()
		point_ = map(float, strs.split())
		point = zip(*(iter(point_),) * no_params)
		points.append(point[0])
	# print points
	return points



def read2DPointsFromTextFile(filename):
	f_h = open(filename,'r')
	points = getFloatNumberFromReadLines(f_h, 2)
	return points


def read2DPointsFromStringReadLines(strs):
	points = getFloatNumberFromStringReadLines(strs, 2)
	return points

def transformCloud(test_cloud, tm):
	H_points = [[p[0], p[1],1] for p in test_cloud]
	M_points = np.matrix(H_points)
	point_t = tm *M_points.getT()
	point_t = point_t.getT().tolist()
	point_t = [[p[0], p[1]] for p in point_t]
	return point_t	

def createTransfromFromXYYaw(x,y,yaw):
	return np.matrix([[math.cos(yaw),-math.sin(yaw),x],[math.sin(yaw),math.cos(yaw),y],[0,0,1]])

def getVertexFromGraph(f_handle, no_vertex, processed=False):
	f_content = f_handle.readlines()
	points = []
	# for str_ in f_content:
	for i in range(0,no_vertex):
		strs = f_content[i].strip()
		strs_splitted =  strs.split()
		noid = int(strs_splitted[1])
		if processed:
			coord = [-float(strs_splitted[2]), float(strs_splitted[3]), -float(strs_splitted[4])]
		else:
			coord = [float(strs_splitted[2]), float(strs_splitted[3]), float(strs_splitted[4])]
		points.append([noid]+coord)
	return points

def getVertexFromGraphAutoCount(f_handle,  processed=False):
	f_content = f_handle.readlines()
	points = []
	# for str_ in f_content:
	for i in range(0,len(f_content)):
		strs = f_content[i].strip()
		strs_splitted =  strs.split()
		# print strs_splitted
		if strs_splitted[0] == "VERTEX" or strs_splitted[0] == "VERTEX2":
			noid = int(strs_splitted[1])
			if processed:
				coord = [-float(strs_splitted[2]), float(strs_splitted[3]), -float(strs_splitted[4])]
			else:
				coord = [float(strs_splitted[2]), float(strs_splitted[3]), float(strs_splitted[4])]
			points.append([noid]+coord)
	return points


def plotMapTransformed(no, ax, tm, map_xx, map_yy):
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_filtered_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-174721/scan_filtered_'+str(no)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_direct_'+str(no+1)+'.txt')
	# test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/alphard/medialink/20150918-180619/scan_lm_filtered_'+str(no)+'.txt')

	test_cloud = read2DPointsFromTextFile('/home/avavav/avdata/mobile_base/skygarden1/scan_filtered_'+str(no)+'.txt')

	point_t = transformCloud(test_cloud, tm)
	map_xx = map_xx + [x[0] for x in point_t]
	map_yy = map_yy + [x[1] for x in point_t]

	ax.set_offsets(np.column_stack((map_xx,map_yy))) 
	return map_xx, map_yy

def accumulateIcpTransform(icp_result, x, y, yaw):
	x_icp =  math.cos(-icp_result[2])*icp_result[0] + math.sin(-icp_result[2])*icp_result[1] 
	y_icp = -math.sin(-icp_result[2])*icp_result[0] + math.cos(-icp_result[2])*icp_result[1]
	x_icp_g =  math.cos(-yaw)*x_icp + math.sin(-yaw)*y_icp + x
	y_icp_g = -math.sin(-yaw)*x_icp + math.cos(-yaw)*y_icp + y
	yaw_icp_g = yaw + icp_result[2]
	return x_icp_g, y_icp_g, yaw_icp_g

def rotate2D(x,y,yaw):
	return x*math.cos(yaw) - y*math.sin(yaw) , x*math.sin(yaw) + y*math.cos(yaw)



class AnimatedScatter(object):
	def setup_anim(self, interval=80):
		self.fig, self.ax = plt.subplots()
		# self.ax.axis([-200, 100, -100, 200])
		# self.ax.axis([-50, 50, -50, 50])
		if self.axis_limit is not None:
			self.ax.axis(self.axis_limit)
		self.ax.set_aspect('equal','datalim')
		self.ani = animation.FuncAnimation(self.fig, self.update, interval=interval, init_func=self.setup_plot, blit=True, frames=len(self.ccscan.poses), repeat=False)
		# self.ani = animation.FuncAnimation(self.fig, self.update, interval=80, init_func=self.setup_plot, blit=True, frames=2000, repeat=False)
		# self.ani = animation.FuncAnimation(self.fig, self.update, interval=80,  blit=True, frames=len(points)-1, repeat=False)
	def __init__(self, ccscan, g_thresh = 10, axis_limit = None):
		# self.no_frame = 0
		self.axis_limit = axis_limit
		self.points_map_x = []
		self.points_map_y = []
		self.travelled_dist = 0 
		self.last_x = 0
		self.last_y = 0
		self.last_yaw = 0
		self.next_capture_dist = 0- g_thresh
		self._to_clear_2 = None
		self.g_thresh = g_thresh
		self.ccscan = ccscan
	def setup_plot(self):
		print 'hi setup anim'
		self.scatter_scan = self.ax.scatter ([],[], color='blue', s=8)
		self.scatter_map = self.ax.scatter ([],[], color='green', s=.4)
		return self.scatter_scan, self.scatter_map
	def update(self,i):
		print 'updating figure...', i
		# print self.no_frame
		# _to_clear = plotScan(self.no_frame)
		# print computeICPBetweenScans(i,i+1)
		# print points[i+1]
		# publishScan(i)

		# print points[i]
		points = self.ccscan.getPose(i)
		yaw = points[2]
		x = points[0]
		y = points[1]
		# if i == 0:
			# yaw = 0
			# x = 0
			# y = 0
		# else:
			# icp_result = computeICPBetweenScans(i-1,i)
			# x,y,yaw = accumulateIcpTransform(icp_result,self.last_x,self.last_y,self.last_yaw)
		self.travelled_dist = self.travelled_dist + math.hypot(self.last_x - x, self.last_y - y)
		# print self.travelled_dist, self.next_capture_dist
		self.last_x = x
		self.last_y = y
		self.last_yaw = yaw

		# tranformation_matrix = np.matrix([[math.cos(-yaw),math.sin(-yaw),x],[-math.sin(-yaw),math.cos(-yaw),y],[0,0,1]])
		_to_clear = [self.scatter_scan]
		# _to_clear.append(plotScan(i, self.scatter_scan))
		# plotScan(i, self.scatter_scan)
		# plotScanTransformed(i, self.scatter_scan, tranformation_matrix)
		ascan = self.ccscan.getCloudTransformedFromIndex(i,separate_axes = True)
		self.scatter_scan.set_offsets(np.column_stack((ascan[0],ascan[1])))
		if self.travelled_dist > self.next_capture_dist:
			while self.travelled_dist > self.next_capture_dist:
				self.next_capture_dist = self.next_capture_dist + self.g_thresh
			# if self._to_clear_2 is not None:
			# self.points_map_x, self.points_map_y = plotMapTransformed(i, self.scatter_map, tranformation_matrix, self.points_map_x, self.points_map_y)
			# self._to_clear_2 = _to_clear_2
			self.ccscan.addScanToCurrentBuiltMap(i)
		# else:
			# self.scatter_map.set_offsets(np.column_stack((self.points_map_x, self.points_map_y)))
		ascan = self.ccscan.getCurrentBuiltMap()
		self.scatter_map.set_offsets(np.column_stack((ascan[0], ascan[1])))
			# self.no_frame = self.no_frame + 1
			# return _to_clear_1, _to_clear_2
		_to_clear = _to_clear + [self.scatter_map]
		return _to_clear


class ICPScan(object):
	def __init__(self, dir_prefix, load_map=False, for_anim=False, axis_limit = None , use_accumulated  = False, load_orig = True):
		if not use_accumulated:
			self.f_handle_poses = open(os.path.join(dir_prefix , 'icp_poses.txt'),'r')
			# self.poses = getFloatNumberFromReadLines(self.f_handle_poses, 12)
			self.poses = getFloatNumberListFromReadLines(self.f_handle_poses)
			self.poses = [list(pose) for pose in self.poses]
		else:
			self.f_handle_poses = open(os.path.join(dir_prefix , 'icp_lm_poses.txt'),'r')
			self.poses = getFloatNumberListFromReadLines(self.f_handle_poses) #, 13)
			# self.poses = [list(pose) for pose in self.poses]
			if load_orig:
				self.f_handle_poses_orig = open(os.path.join(dir_prefix , 'icp_poses.txt'),'r')
				# self.poses_orig = getFloatNumberFromReadLines(self.f_handle_poses_orig, 12)
				self.poses_orig = getFloatNumberListFromReadLines(self.f_handle_poses_orig)
				self.poses_orig = [list(pose) for pose in self.poses_orig]
		self.dir_prefix = dir_prefix
		self.use_accumulated = use_accumulated
		self.load_orig = load_orig
		if load_map:
			self.f_handle_open_map = open(os.path.join(dir_prefix, 'map.txt'), 'rb')

			data_map = self.f_handle_open_map.read()
			file_openmap = ZipInputStream(StringIO.StringIO(data_map))
			lines_map = file_openmap.readlines()
			self.map_cloud = read2DPointsFromStringReadLines(lines_map)
		if for_anim:
			self.points_map_x = []
			self.points_map_y = []
			self.animate_obj = AnimatedScatter(self, axis_limit=axis_limit)
		self.corrected_edges = {}

	def setAnimReady(self):
		self.animate_obj.setup_anim()

	def getCloudFromIndex(self, no, filtered=False,separate_axes=False):
		if self.use_accumulated:
			_tcloud = read2DPointsFromTextFile(os.path.join(self.dir_prefix , 'scan_lm_filtered_'+str(no)+'.txt'))
		else:
			if filtered:
				_tcloud = read2DPointsFromTextFile(os.path.join(self.dir_prefix , 'scan_filtered_'+str(no+1)+'.txt'))
			else:
				_tcloud = read2DPointsFromTextFile(os.path.join(self.dir_prefix , 'scan_'+str(no+1)+'.txt'))
		if separate_axes:
			return [c[0] for c in _tcloud], [c[1] for c in _tcloud]
		else:
			return _tcloud

	def getCloudTransformedFromIndex(self, no, x=None, y=None, yaw=None, filtered=False, additive_pose=False, separate_axes=False):
		if x is None:
			x = self.poses[no][0]
		elif additive_pose:
			x = x + self.poses[no][0]
		if y is None:
			y = self.poses[no][1]
		elif additive_pose:
			y = y + self.poses[no][1]
		if yaw is None:
			yaw = self.poses[no][2]
		elif additive_pose:
			yaw = yaw + self.poses[no][2]
		if self.use_accumulated:
			_tcloud = transformCloud(read2DPointsFromTextFile(os.path.join(self.dir_prefix , 'scan_lm_filtered_'+str(no)+'.txt')), createTransfromFromXYYaw(x,y,yaw))
		else:
			if filtered:
				_tcloud = transformCloud(read2DPointsFromTextFile(os.path.join(self.dir_prefix , 'scan_filtered_'+str(no+1)+'.txt')), createTransfromFromXYYaw(x,y,yaw))
			else:
				_tcloud = transformCloud(read2DPointsFromTextFile(os.path.join(self.dir_prefix , 'scan_'+str(no+1)+'.txt')), createTransfromFromXYYaw(x,y,yaw))
		if separate_axes:
			return [c[0] for c in _tcloud], [c[1] for c in _tcloud]
		else:
			return _tcloud

	def getMap(self, separate_axes=False):
		if separate_axes:
			return [c[0] for c in self.map_cloud], [c[1] for c in self.map_cloud]
		else:
			return self.map_cloud

	def addScanToCurrentBuiltMap(self, no, filtered=True):
		new_map_x, new_map_y = self.getCloudTransformedFromIndex(no, filtered=filtered, separate_axes=True)
		self.points_map_x = self.points_map_x + new_map_x
		self.points_map_y = self.points_map_y + new_map_y

	def getCurrentBuiltMap(self):
		return self.points_map_x, self.points_map_y

	def getPose(self,no, use_original = False):
		if  use_original:
			# print no, len(self.poses_orig)
			return self.poses_orig[no]
		else:
			return self.poses[no]

	def modPose(self,no, new_pose, additive_pose=False, with_original=False):
		orig_pose = self.poses[no][0:3]
		if additive_pose:
			for i in range(3):
				self.poses[no][i] = self.poses[no][i] + new_pose[i]
		else:
			for i in range(3):
				self.poses[no][i] =  new_pose[i]
		if self.use_accumulated and with_original:
			delta_pose = [-o +a for o,a in zip(orig_pose[0:3], self.poses[no][0:3])]
			indices = self.getOriginalIndexFromAccumulatedIndex(no)
			anchor_index = indices[-1]
			for ii in indices:
				self.poses_orig[ii][2] = self.poses_orig[ii][2] + delta_pose[2]
				_x, _y = rotate2D(self.poses_orig[ii][0] - self.poses_orig[anchor_index][0]
						, self.poses_orig[ii][1] - self.poses_orig[anchor_index][1]
						,delta_pose[2])
				self.poses_orig[ii][0] = _x+ delta_pose[0] + self.poses_orig[anchor_index][0]
				self.poses_orig[ii][1] = _y+ delta_pose[1] + self.poses_orig[anchor_index][1]


	def computeICPBetweenScans(self,no1,no2, init_x = 0 , init_y = 0, init_yaw = 0):
		example = cython_catkin_example.PyCCExample()
		test_cloud = self.getCloudFromIndex(no1)
		test_cloud2 = self.getCloudFromIndex(no2)
		cython_icp_result = example.processICP(np.array([x[0] for x in test_cloud], np.float32),np.array([x[1] for x in test_cloud], np.float32),np.array([x[0] for x in test_cloud2], np.float32),np.array([x[1] for x in test_cloud2], np.float32), init_x, init_y, init_yaw)
		return cython_icp_result

	def getInitialValues(self,no1,no2):
		no1_pose = self.poses[no1]
		no2_pose = self.poses[no2]
		x = no2_pose[0] - no1_pose[0]
		y = no2_pose[1] - no1_pose[1]
		yaw = no2_pose[2] - no1_pose[2]
		return x,y,yaw
	def recorrectPoseUseICP(self,no1,no2):
		init_pose = self.getInitialValues(no1,no2)
		new_rel = self.computeICPBetweenScans(no1,no2,init_x=init_pose[0], init_y=init_pose[1], init_yaw=init_pose[2])
		self.poses[no2][0] = new_rel[0] + self.poses[no1][0]
		self.poses[no2][1] = new_rel[1] + self.poses[no1][1]
		self.poses[no2][2] = new_rel[2] + self.poses[no1][2]
		self.corrected_edges[(no1,no2)] = new_rel

	def putLastEdgeStringIntoGraphFile(self):
		if self.use_original_interwoven:
			fh = open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'a')
			fh.write (self.last_created_edge_string)
			fh.close()
		else:
			fh = open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'a')
			fh.write (self.last_created_edge_string)
			fh.close()


	def createEdgeString(self, no1,no2, use_original_interwoven=False):
		self.use_original_interwoven = use_original_interwoven
		if self.use_accumulated and use_original_interwoven:
			print no1, no2
			if (no1,no2) in self.corrected_edges:
				new_rel = self.corrected_edges[(no1,no2)]
			else:
				new_rel = self.getInitialValues(no1,no2)
			orig_indices1 = map(int,self.getPose(no1)[12:])#.reverse()
			orig_indices2 = map(int,self.getPose(no2)[12:])#.reverse()
			# print orig_indices1, orig_indices2
			# print self.getPose(no1), self.getPose(no2)
			# print self.getPose(orig_indices1[-1], use_original=True)
			# print self.getPose(orig_indices2[-1], use_original=True)
			pose_1 = self.getPose(no1)
			pose_2 = self.getPose(no2)
			str_edge_added = ''
			for indices2 in orig_indices2:
				this_index2_pose = self.getPose(indices2, use_original=True)
				for indices1 in orig_indices1:
					this_index1_pose = self.getPose(indices1, use_original=True)
					delta_x = new_rel[0] -(pose_2[0] - this_index2_pose[0]) + (pose_1[0]  - this_index1_pose[0])
					delta_y = new_rel[1] -(pose_2[1] - this_index2_pose[1]) + (pose_1[1]  - this_index1_pose[1])
					delta_yaw = new_rel[2] -(pose_2[2] - this_index2_pose[2]) + (pose_1[2]  - this_index1_pose[2])
					if (no1,no2) in self.corrected_edges:
						str_edge_added = str_edge_added + createToroEdgeString(indices2+1, indices1+1,  delta_x, delta_y, delta_yaw, new_rel[3], new_rel[4], new_rel[5], new_rel[6], new_rel[7], new_rel[8])
					else:
						str_edge_added = str_edge_added + createToroEdgeString(indices2+1, indices1+1,  delta_x, delta_y, delta_yaw, 1, 0, 1, 1, 0, 0)
			self.last_created_edge_string = str_edge_added
			return str_edge_added
		else:
			if (no1,no2) in self.corrected_edges:
				new_rel = self.corrected_edges[(no1,no2)]
				str_edge_added = ''
				str_edge_added = str_edge_added + 'EDGE2 '
				str_edge_added = str_edge_added + format(no2+1,'d') + ' '
				str_edge_added = str_edge_added + format(no1+1,'d') + ' '
				str_edge_added = str_edge_added + format(new_rel[0],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[1],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[2],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[3],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[4],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[5],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[6],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[7],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[8],'.4f') + ' '
				str_edge_added = str_edge_added + '\n'
			else:
				new_rel = self.getInitialValues(no1,no2)
				str_edge_added = ''
				str_edge_added = str_edge_added + 'EDGE2 '
				str_edge_added = str_edge_added + format(no2+1,'d') + ' '
				str_edge_added = str_edge_added + format(no1+1,'d') + ' '
				str_edge_added = str_edge_added + format(new_rel[0],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[1],'.4f') + ' '
				str_edge_added = str_edge_added + format(new_rel[2],'.4f') + ' 1 0 1 1 0 0'
				str_edge_added = str_edge_added + '\n'
			self.last_created_edge_string = str_edge_added
			return str_edge_added

	def getOriginalPosesFromAccumulatedIndex(self, no):
		orig_indices = map(int,self.getPose(no)[12:])#.reverse()
		points = []
		for ind in orig_indices:
			points.append(self.getPose(ind, use_original = True))
		return points, orig_indices

	def getOriginalIndexFromAccumulatedIndex(self, no):
		return map(int,self.getPose(no)[12:])

class MapScan(ICPScan):
	def __init__(self, dir_prefix, use_accumulated = False):
		super(MapScan, self).__init__(dir_prefix, use_accumulated = use_accumulated)
		self.vertices = []
		self.edges = {}

	def createVertexStringFromPoses(self, include_zero_vertex = True, start_index = 1, use_original = False, use_original_index = False):
		if include_zero_vertex:
			self.last_str_vertex = 'VERTEX2 0 0 0 0\n'
		else:
			self.last_str_vertex = ''
		if (self.use_accumulated and use_original):
			vertex_ii = 0
			for n in range(0,len(self.poses)):
				this_poses , this_poses_original_indices = self.getOriginalPosesFromAccumulatedIndex(n)
				for i in range(0, len(this_poses)):
					if use_original_index:
						self.last_str_vertex = self.last_str_vertex + createToroVertexString(this_poses_original_indices[i], this_poses[i][0], this_poses[i][1], this_poses[i][2])
					else:
						self.last_str_vertex = self.last_str_vertex + createToroVertexString(vertex_ii+start_index, this_poses[i][0], this_poses[i][1], this_poses[i][2])
					vertex_ii = vertex_ii + 1
		else:
			for n in range(0,len(self.poses)):
				self.last_str_vertex = self.last_str_vertex + createToroVertexString(n+start_index, self.poses[n][0], self.poses[n][1], self.poses[n][2])
		return self.last_str_vertex

	def createVertexStringFromVertices(self, include_zero_vertex = True, start_index = 1,  use_original_index = True):
		if include_zero_vertex:
			self.last_str_vertex = 'VERTEX2 0 0 0 0\n'
		else:
			self.last_str_vertex = ''
		for n in range(0,len(self.vertices)):
			if use_original_index:
				self.last_str_vertex = self.last_str_vertex + createToroVertexString(self.vertices[n][0], self.vertices[n][1], self.vertices[n][2], self.vertices[n][3])
			else:
				self.last_str_vertex = self.last_str_vertex + createToroVertexString(n+start_index, self.vertices[n][1], self.vertices[n][2], self.vertices[n][3])
		return self.last_str_vertex


	def createEdgeStringFromEdges(self):
		self.last_str_edge = ''
		for key, value in collections.OrderedDict(sorted(self.edges.items())).iteritems():
			self.last_str_edge = self.last_str_edge + createToroEdgeString(key[1], key[0], value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7], value[8])
			# if key[1] == 656 and key[0] == 655:
				# print value
				# print createToroEdgeString(key[1], key[0], value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7], value[8])
		return self.last_str_edge

	def getAllOriginalPoses(self):
		poses_orig_result = []
		for n in range(0,len(self.poses)):
			this_poses , this_poses_original_indices = self.getOriginalPosesFromAccumulatedIndex(n)
			for tp , tpi in zip(this_poses, this_poses_original_indices):
				tp.append(tpi)
				poses_orig_result.append(tp)
		return poses_orig_result

	def loadGraphVertices(self, use_original = False, remove_zero_vertex = True, use_processed_map = False, add_to_current = False, reverse_processed_map = False):
		if self.use_accumulated and not use_original:
			if use_processed_map:
				vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_lm_poses-treeopt-final.graph'), 'r'), processed = reverse_processed_map)
			else:
				vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'r'))
		else:
			if use_processed_map:
				vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_poses-treeopt-final.graph'), 'r'), processed = reverse_processed_map)
			else:
				vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'r'))
		if remove_zero_vertex:
			index_zero = [int(x[0]) for x in vertices].index(0)
			del vertices[index_zero]
		if add_to_current:
			self.vertices = self.vertices + vertices
		else:
			self.vertices = vertices
		return self.vertices

	def loadGraphEdges(self, use_original = False,  use_processed_map = False, add_to_current = False, reverse_processed_map = False, override_covariances = False):
		if self.use_accumulated and not use_original:
			if use_processed_map:
				edges = getEdgeFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_lm_poses-treeopt-final.graph'), 'r'), processed = reverse_processed_map, override_covariances = override_covariances)
			else:
				edges = getEdgeFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'r'), override_covariances = override_covariances)
		else:
			if use_processed_map:
				edges = getEdgeFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_poses-treeopt-final.graph'), 'r'), processed = reverse_processed_map, override_covariances = override_covariances)
			else:
				edges = getEdgeFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'r'), override_covariances = override_covariances)
		if add_to_current:
			self.edges.update(edges)
		else:
			self.edges = edges
		return self.edges

	def saveGraphVertices(self, use_original = False):
		if self.use_accumulated and not use_original:
			vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'r'))
			fh_str = open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'r').readlines()
			del fh_str[0:len(vertices)]
			# fh_str = fh_str + self.last_str_vertex.splitlines()
			# open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'w').writelines(fh_str)
			fh_str = self.last_str_vertex + "".join(fh_str)
			open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'w').write(fh_str)

		else:
			vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'r'))
			fh_str = open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'r').readlines()
			del fh_str[0:len(vertices)]
			fh_str = self.last_str_vertex + "".join(fh_str)
			# print fh_str
			open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'w').write(fh_str)
		# return self.vertices

	def saveGraphEdges(self, use_original = False):
		if self.use_accumulated and not use_original:
			vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'r'))
			fh_str = open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'r').readlines()
			del fh_str[len(vertices):]
			# fh_str = fh_str + self.last_str_vertex.splitlines()
			# open(os.path.join(self.dir_prefix,'icp_lm_poses.graph'), 'w').writelines(fh_str)
			fh_str = "".join(fh_str) + self.last_str_edge 
			open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'w').write(fh_str)
		else:
			vertices = getVertexFromGraphAutoCount(open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'r'))
			fh_str = open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'r').readlines()
			del fh_str[len(vertices):]
			fh_str = "".join(fh_str) + self.last_str_edge 
			open(os.path.join(self.dir_prefix,'icp_poses.graph'), 'w').write(fh_str)

	def appendVerticesWithSetPoses(self, poses, start_id = 1, include_zero_vertex = False, use_original_index = False, reject_duplicate = True):
		if use_original_index:
			assert len(poses[0]) > 12 , "no vertex ids can be inferred"
		mode_use_index = False
		if len(poses[0]) < 12:
			mode_use_index = True
		if include_zero_vertex:
			self.vertices = self.vertices + [0,  0,  0,  0]
		index_pose = 0
		if reject_duplicate:
			_ids = [x[0] for x in self.vertices]
			for pose in poses:
				if not use_original_index:
					if not index_pose + start_id  in _ids:
						self.vertices.append([index_pose + start_id ,pose[0] ,pose[1] ,pose[2]])
				else:
					if not int(pose[-1]) in _ids:
						self.vertices.append([int(pose[-1]), pose[0], pose[1], pose[2]])
				index_pose = index_pose + 1
		else:
			for pose in poses:
				if not use_original_index:
					self.vertices.append([index_pose + start_id ,pose[0] ,pose[1] ,pose[2]])
				else:
					self.vertices,append([int(pose[-1]), pose[0], pose[1], pose[2]])
				index_pose = index_pose + 1
		return self.vertices

	def getRelativePoseBetweenVertices(self,no1,no2):
		no1_pose = self.vertices[no1]
		no2_pose = self.vertices[no2]
		x = no2_pose[1] - no1_pose[1]
		y = no2_pose[2] - no1_pose[2]
		yaw = no2_pose[3] - no1_pose[3]
		return [x,y,yaw], no1_pose[0], no2_pose[0]

	def transformVectorToVerticeCoord(self,no, pose_x, pose_y):
		pose_ver = self.vertices[no]
		x = math.cos( -pose_ver[3] ) * pose_x - math.sin( -pose_ver[3] ) * pose_y
		y = math.sin( -pose_ver[3] ) * pose_x + math.cos( -pose_ver[3] ) * pose_y
		return x, y

	def appendEdgesWithVertices(self, overwrite_duplicate = True, use_information_matrix = False):
		index_vertex = 1
		new_edges = {}
		for vertex in self.vertices[1:]:
			rel_pose, observing_id, observed_id = self.getRelativePoseBetweenVertices(index_vertex-1, index_vertex)
			x_t , y_t = self.transformVectorToVerticeCoord(index_vertex - 1, rel_pose[0], rel_pose[1])
			rel_pose[0] = x_t
			rel_pose[1] = y_t
			new_edges[(observing_id, observed_id)] = rel_pose + [1.,0.,1.,1.,0.,0.]
			# new_edges[(observing_id, observed_id)] = [ x_t, y_t , rel_pose[2]] + [1.,0.,1.,1.,0.,0.]
			if overwrite_duplicate:
				self.edges[(observing_id, observed_id)] = rel_pose + [1.,0.,1.,1.,0.,0.]
			else:
				if not (observing_id, observed_id) in self.edges:
					self.edges[(observing_id, observed_id)] = rel_pose + [1.,0.,1.,1.,0.,0.]
				# else:
					# print "duplicate detected...", (observed_id, observed_id)
			index_vertex = index_vertex + 1
		return new_edges

	def getIndexFromVertexId (self, vid):
		vertices_indices = [v[0] for v in self.vertices]
		return vertices_indices.index(vid)

	def getVertexFromVertexId (self, vid):
		return self.vertices[self.getIndexFromVertexId(vid)]

	def manuallyAddEdge(self,  no1, no2, overwrite_duplicate = True, use_information_matrix = False, override_information=None):
		vertices_indices = [v[0] for v in self.vertices]
		index_no1 = vertices_indices.index(no1)
		index_no2 = vertices_indices.index(no2)
		print index_no1, index_no2, self.vertices[index_no1], self.vertices[index_no2]
		rel_pose, observing_id, observed_id = self.getRelativePoseBetweenVertices(index_no1, index_no2)
		x_t , y_t = self.transformVectorToVerticeCoord(index_no1, rel_pose[0], rel_pose[1])
		rel_pose[0] = x_t
		rel_pose[1] = y_t
		print rel_pose
		if overwrite_duplicate:
			if override_information is not None:
				self.edges[(observing_id, observed_id )] = rel_pose + [override_information,0.,override_information,override_information,0.,0.]
			else:
				self.edges[(observing_id, observed_id )] = rel_pose + [1.,0.,1.,1.,0.,0.]
		else:
			if not (observing_id, observed_id ) in self.edges:
				if override_information is not None:
					self.edges[(observing_id, observed_id )] = rel_pose + [override_information,0.,override_information,override_information,0.,0.]
				else:
					self.edges[(observing_id, observed_id )] = rel_pose + [1.,0.,1.,1.,0.,0.]

class ZipInputStream:

	def __init__(self, file):
		self.file = file
		self.__rewind()

	def __rewind(self):
		self.zip = zlib.decompressobj()
		self.pos = 0 # position in zipped stream
		self.offset = 0 # position in unzipped stream
		self.data = ""

	def __fill(self, bytes):
		if self.zip:
			# read until we have enough bytes in the buffer
			while not bytes or len(self.data) < bytes:
				self.file.seek(self.pos)
				data = self.file.read(16384)
				if not data:
					self.data = self.data + self.zip.flush()
					self.zip = None # no more data
					break
				self.pos = self.pos + len(data)
				self.data = self.data + self.zip.decompress(data)

	def seek(self, offset, whence=0):
		if whence == 0:
			position = offset
		elif whence == 1:
			position = self.offset + offset
		else:
			raise IOError, "Illegal argument"
		if position < self.offset:
			raise IOError, "Cannot seek backwards"

		# skip forward, in 16k blocks
		while position > self.offset:
			if not self.read(min(position - self.offset, 16384)):
				break

	def tell(self):
		return self.offset

	def read(self, bytes = 0):
		self.__fill(bytes)
		if bytes:
			data = self.data[:bytes]
			self.data = self.data[bytes:]
		else:
			data = self.data
			self.data = ""
		self.offset = self.offset + len(data)
		return data

	def readline(self):
		# make sure we have an entire line
		while self.zip and "\n" not in self.data:
			self.__fill(len(self.data) + 512)
		i = string.find(self.data, "\n") + 1
		if i <= 0:
			return self.read()
		return self.read(i)

	def readlines(self):
		lines = []
		while 1:
			s = self.readline()
			if not s:
				break
			lines.append(s)
		return lines
