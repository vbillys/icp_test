import os
import numpy as np
import math
from ZipLib import ZipInputStream
import string, StringIO
import pylab as plt
import matplotlib.animation as animation
from cython_catkin_example import cython_catkin_example

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
	def __init__(self, dir_prefix, load_map=False, for_anim=False, axis_limit = None , use_accumulated  = False):
		if not use_accumulated:
			self.f_handle_poses = open(os.path.join(dir_prefix , 'icp_poses.txt'),'r')
			self.poses = getFloatNumberFromReadLines(self.f_handle_poses, 12)
			self.poses = [list(pose) for pose in self.poses]
		else:
			self.f_handle_poses = open(os.path.join(dir_prefix , 'icp_lm_poses.txt'),'r')
			self.poses = getFloatNumberListFromReadLines(self.f_handle_poses) #, 13)
			# self.poses = [list(pose) for pose in self.poses]
		self.dir_prefix = dir_prefix
		self.use_accumulated = use_accumulated
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

	def getPose(self,no):
		return self.poses[no]

	def modPose(self,no, new_pose, additive_pose=False):
		if additive_pose:
			for i in range(3):
				self.poses[no][i] = self.poses[no][i] + new_pose[i]
		else:
			for i in range(3):
				self.poses[no][i] =  new_pose[i]

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

	def createEdgeString(self, no1,no2):
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
		return str_edge_added


