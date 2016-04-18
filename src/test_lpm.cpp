/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/gui.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/types.h>
#include <mrpt/poses/CPose2D.h>

#include <iostream>
#include <fstream>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <icp_test/processICP.h>
#include <tf/transform_broadcaster.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include <cassert>
#include <iostream>
#include <fstream>

#include "treeoptimizer2.hh"
#include <boost/format.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

#include <feature/BetaGrid.h>
#include <feature/CurvatureDetector.h>
#include <utils/SimpleMinMaxPeakFinder.h>

#include <Eigen/Dense>

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

ros::Publisher  pub, pub_odom;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;
using namespace PointMatcherSupport;


typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
PM::ICP icp;

bool skip_window=false;
int  ICP_method = (int) icpClassic;


#define SCANS_SIZE 361

#if 0
// Hard matching in a corridor
float SCAN_RANGES_1[] = { 2.670f,2.670f,2.670f,2.670f,2.670f,2.660f,2.670f,2.670f,2.670f,2.670f,2.670f,2.670f,2.670f,2.670f,2.670f,2.670f,2.680f,2.680f,2.680f,2.680f,2.690f,2.690f,2.700f,2.690f,2.700f,2.700f,2.710f,2.710f,2.720f,2.720f,2.720f,2.730f,2.740f,2.730f,2.740f,2.750f,2.750f,2.760f,2.770f,2.780f,2.780f,2.790f,2.800f,2.800f,2.820f,2.820f,2.830f,2.840f,2.850f,2.860f,2.870f,2.880f,2.890f,2.900f,2.910f,2.920f,2.930f,2.940f,2.950f,2.970f,2.990f,2.990f,3.010f,3.020f,3.040f,3.050f,3.070f,3.080f,3.100f,3.120f,3.140f,3.150f,3.170f,3.180f,3.200f,3.220f,3.240f,3.250f,3.280f,3.300f,3.320f,3.340f,3.370f,3.390f,3.420f,3.440f,3.470f,3.490f,3.510f,3.540f,3.570f,3.590f,3.630f,3.650f,3.690f,3.710f,3.750f,3.770f,3.820f,3.850f,3.890f,3.920f,3.960f,3.990f,4.030f,4.060f,4.120f,4.160f,4.210f,4.230f,4.290f,4.310f,4.380f,4.420f,4.490f,4.530f,4.590f,4.640f,4.710f,4.760f,4.830f,4.890f,4.970f,5.010f,5.110f,5.170f,5.260f,5.330f,5.410f,5.490f,5.590f,5.670f,5.780f,5.860f,5.980f,6.070f,6.180f,6.300f,6.410f,6.530f,6.670f,6.790f,6.950f,7.100f,7.250f,7.400f,7.590f,7.750f,7.940f,8.140f,8.380f,8.580f,8.840f,9.060f,9.360f,9.610f,9.940f,10.210f,10.600f,10.930f,11.380f,11.750f,12.280f,12.730f,13.300f,13.850f,14.580f,15.210f,16.040f,16.820f,17.890f,18.840f,20.210f,21.470f,23.260f,81.870f,81.870f,81.910f,81.910f,81.910f,39.910f,39.940f,81.870f,	81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.870f,81.910f,81.870f,81.870f,21.120f,21.120f,21.180f,21.190f,21.250f,19.140f,21.310f,14.710f,14.170f,13.680f,13.130f,12.690f,12.230f,11.860f,11.450f,11.140f,10.800f,10.520f,10.180f,9.920f,9.650f,9.440f,9.170f,8.980f,8.740f,8.570f,8.360f,8.180f,7.990f,7.850f,7.660f,7.540f,7.380f,7.260f,7.110f,6.990f,6.860f,6.760f,6.620f,6.530f,6.410f,6.310f,6.200f,6.120f,6.020f,5.940f,5.840f,5.780f,5.680f,5.620f,5.520f,5.460f,5.380f,5.320f,5.250f,5.200f,5.130f,5.080f,5.010f,4.960f,4.900f,4.860f,4.790f,4.760f,4.700f,4.660f,4.610f,4.570f,4.520f,4.490f,4.430f,4.400f,4.350f,4.320f,4.280f,4.240f,4.200f,4.180f,4.140f,4.120f,4.070f,4.050f,4.010f,3.990f,3.950f,3.930f,3.900f,3.870f,3.850f,3.820f,3.790f,3.770f,3.740f,3.720f,3.700f,3.670f,3.650f,3.640f,3.610f,3.590f,3.570f,3.560f,3.530f,3.520f,3.500f,3.490f,3.460f,3.450f,3.430f,3.420f,3.400f,3.400f,3.380f,3.360f,3.360f,3.330f,3.320f,3.310f,3.300f,3.290f,3.270f,3.260f,3.250f,3.240f,3.230f,3.220f,3.210f,3.200f,3.200f,3.190f,3.180f,3.170f,3.160f,3.150f,3.150f,3.150f,3.140f,3.140f,3.120f,3.120f,3.110f,3.100f,3.100f,3.100f,3.100f,3.090f,3.080f,3.080f,3.080f,3.080f,3.070f,3.070f,3.070f,3.060f,3.070f,3.060f,3.060f };
char  SCAN_VALID_1[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

float SCAN_RANGES_2[] = {2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.630f,2.640f,2.640f,2.640f,2.640f,2.640f,2.640f,2.650f,2.650f,2.650f,2.650f,2.660f,2.660f,2.670f,2.670f,2.670f,2.670f,2.680f,2.680f,2.690f,2.690f,2.700f,2.710f,2.710f,2.710f,2.720f,2.730f,2.740f,2.750f,2.750f,2.760f,2.770f,2.780f,2.790f,2.800f,2.800f,2.820f,2.830f,2.840f,2.850f,2.860f,2.870f,2.880f,2.890f,2.900f,2.920f,2.930f,2.930f,2.950f,2.960f,2.980f,2.990f,3.000f,3.020f,3.040f,3.060f,3.070f,3.090f,3.110f,3.120f,3.140f,3.150f,3.170f,3.190f,3.200f,3.230f,3.250f,3.270f,3.290f,3.320f,3.330f,3.360f,3.380f,3.410f,3.430f,3.460f,3.490f,3.510f,3.540f,3.570f,3.590f,3.620f,3.650f,3.680f,3.720f,3.750f,3.750f,3.800f,3.850f,3.880f,3.930f,3.960f,3.990f,4.040f,4.060f,4.110f,4.160f,4.210f,4.250f,4.300f,4.330f,4.410f,4.450f,4.500f,4.550f,4.610f,4.660f,4.730f,4.780f,4.860f,4.930f,4.990f,5.060f,5.140f,5.210f,5.300f,5.380f,5.450f,5.540f,5.630f,5.730f,5.820f,5.920f,6.030f,6.130f,6.240f,6.360f,6.490f,6.620f,6.760f,6.900f,7.050f,7.210f,7.370f,7.540f,7.730f,7.920f,8.120f,8.330f,8.550f,8.780f,9.040f,9.300f,9.580f,9.880f,10.230f,10.580f,10.940f,11.340f,11.780f,12.240f,12.740f,13.300f,13.910f,14.580f,15.260f,16.030f,16.930f,17.900f,19.060f,20.300f,81.870f,23.500f,81.910f,81.910f,81.910f,81.910f,81.910f,38.760f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.910f,81.870f,81.910f,20.620f,20.000f,20.020f,20.050f,20.070f,17.870f,20.140f,17.950f,20.210f,13.560f,13.160f,12.700f,12.240f,11.850f,11.490f,11.120f,10.840f,10.510f,10.230f,9.940f,9.700f,9.440f,9.220f,8.990f,8.790f,8.570f,8.390f,8.210f,8.030f,7.860f,7.710f,7.560f,7.420f,7.270f,7.150f,7.020f,6.900f,6.770f,6.660f,6.550f,6.460f,6.340f,6.260f,6.150f,6.060f,5.980f,5.890f,5.800f,5.730f,5.650f,5.580f,5.500f,5.430f,5.360f,5.300f,5.230f,5.170f,5.100f,5.060f,5.000f,4.940f,4.880f,4.830f,4.770f,4.730f,4.690f,4.640f,4.600f,4.550f,4.520f,4.480f,4.420f,4.390f,4.340f,4.310f,4.280f,4.240f,4.200f,4.180f,4.150f,4.120f,4.070f,4.050f,4.010f,3.990f,3.960f,3.940f,3.910f,3.880f,3.850f,3.830f,3.800f,3.780f,3.760f,3.740f,3.710f,3.690f,3.660f,3.640f,3.620f,3.600f,3.580f,3.570f,3.550f,3.540f,3.520f,3.510f,3.490f,3.470f,3.450f,3.440f,3.430f,3.410f,3.400f,3.390f,3.380f,3.380f,3.350f,3.350f,3.340f,3.330f,3.290f,3.290f,3.270f,3.260f,3.250f,3.240f,3.240f,3.230f,3.220f,3.210f,3.200f,3.200f,3.180f,3.180f,3.180f,3.180f,3.160f,3.150f,3.150f,3.150f,3.130f,3.130f,3.120f,3.130f,3.120f,3.140f,3.130f,3.130f,3.110f,3.120f,3.120f,3.120f,3.100f,3.100f,3.090f};
char  SCAN_VALID_2[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
#else
// A more usual case:
float SCAN_RANGES_1[] = {0.910f,0.900f,0.910f,0.900f,0.900f,0.890f,0.890f,0.880f,0.890f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.870f,0.880f,0.870f,0.870f,0.870f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.890f,0.880f,0.880f,0.880f,0.890f,0.880f,0.890f,0.890f,0.880f,0.890f,0.890f,0.880f,0.890f,0.890f,0.890f,0.890f,0.890f,0.890f,0.900f,0.900f,0.900f,0.900f,0.900f,0.910f,0.910f,0.910f,0.910f,0.920f,0.920f,0.920f,0.920f,0.920f,0.930f,0.930f,0.930f,0.930f,0.940f,0.940f,0.950f,0.950f,0.950f,0.950f,0.960f,0.960f,0.970f,0.970f,0.970f,0.980f,0.980f,0.990f,1.000f,1.000f,1.000f,1.010f,1.010f,1.020f,1.030f,1.030f,1.030f,1.040f,1.050f,1.060f,1.050f,1.060f,1.070f,1.070f,1.080f,1.080f,1.090f,1.100f,1.110f,1.120f,1.120f,1.130f,1.140f,1.140f,1.160f,1.170f,1.180f,1.180f,1.190f,1.200f,1.220f,1.220f,1.230f,1.230f,1.240f,1.250f,1.270f,1.280f,1.290f,1.300f,1.320f,1.320f,1.350f,1.360f,1.370f,1.390f,1.410f,1.410f,1.420f,1.430f,1.450f,1.470f,1.490f,1.500f,1.520f,1.530f,1.560f,1.580f,1.600f,1.620f,1.650f,1.670f,1.700f,1.730f,1.750f,1.780f,1.800f,1.830f,1.850f,1.880f,1.910f,1.940f,1.980f,2.010f,2.060f,2.090f,2.130f,2.180f,2.220f,2.250f,2.300f,2.350f,2.410f,2.460f,2.520f,2.570f,2.640f,2.700f,2.780f,2.850f,2.930f,3.010f,3.100f,3.200f,3.300f,3.390f,3.500f,3.620f,3.770f,3.920f,4.070f,4.230f,4.430f,4.610f,4.820f,5.040f,5.290f,5.520f,8.970f,8.960f,8.950f,8.930f,8.940f,8.930f,9.050f,9.970f,9.960f,10.110f,13.960f,18.870f,19.290f,81.910f,20.890f,48.750f,48.840f,48.840f,19.970f,19.980f,19.990f,15.410f,20.010f,19.740f,17.650f,17.400f,14.360f,12.860f,11.260f,11.230f,8.550f,8.630f,9.120f,9.120f,8.670f,8.570f,7.230f,7.080f,7.040f,6.980f,6.970f,5.260f,5.030f,4.830f,4.620f,4.440f,4.390f,4.410f,4.410f,4.410f,4.430f,4.440f,4.460f,4.460f,4.490f,4.510f,4.540f,3.970f,3.820f,3.730f,3.640f,3.550f,3.460f,3.400f,3.320f,3.300f,3.320f,3.320f,3.340f,2.790f,2.640f,2.600f,2.570f,2.540f,2.530f,2.510f,2.490f,2.490f,2.480f,2.470f,2.460f,2.460f,2.460f,2.450f,2.450f,2.450f,2.460f,2.460f,2.470f,2.480f,2.490f,2.490f,2.520f,2.510f,2.550f,2.570f,2.610f,2.640f,2.980f,3.040f,3.010f,2.980f,2.940f,2.920f,2.890f,2.870f,2.830f,2.810f,2.780f,2.760f,2.740f,2.720f,2.690f,2.670f,2.650f,2.630f,2.620f,2.610f,2.590f,2.560f,2.550f,2.530f,2.510f,2.500f,2.480f,2.460f,2.450f,2.430f,2.420f,2.400f,2.390f,2.380f,2.360f,2.350f,2.340f,2.330f,2.310f,2.300f,2.290f,2.280f,2.270f,2.260f,2.250f,2.240f,2.230f,2.230f,2.220f,2.210f,2.200f,2.190f,2.180f,2.170f,1.320f,1.140f,1.130f,1.130f,1.120f,1.120f,1.110f,1.110f,1.110f,1.110f,1.100f,1.110f,1.100f};
char  SCAN_VALID_1[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

float SCAN_RANGES_2[] = {0.720f,0.720f,0.720f,0.720f,0.720f,0.720f,0.710f,0.720f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.710f,0.720f,0.720f,0.720f,0.720f,0.730f,0.730f,0.730f,0.730f,0.730f,0.730f,0.730f,0.740f,0.740f,0.740f,0.740f,0.740f,0.740f,0.750f,0.750f,0.750f,0.750f,0.750f,0.750f,0.750f,0.750f,0.760f,0.760f,0.760f,0.760f,0.760f,0.760f,0.760f,0.760f,0.770f,0.770f,0.770f,0.770f,0.780f,0.780f,0.780f,0.790f,0.790f,0.800f,0.800f,0.800f,0.800f,0.800f,0.800f,0.810f,0.810f,0.820f,0.820f,0.830f,0.830f,0.840f,0.840f,0.850f,0.850f,0.860f,0.860f,0.860f,0.870f,0.870f,0.880f,0.890f,0.890f,0.900f,0.900f,0.910f,0.920f,0.930f,0.930f,0.940f,0.940f,0.940f,0.950f,0.960f,0.960f,0.970f,0.980f,0.990f,1.000f,1.010f,1.020f,1.030f,1.040f,1.050f,1.060f,1.070f,1.080f,1.080f,1.100f,1.100f,1.120f,1.120f,1.140f,1.140f,1.170f,1.160f,1.180f,1.190f,1.210f,1.220f,1.240f,1.250f,1.280f,1.290f,1.300f,1.320f,1.340f,1.350f,1.380f,1.390f,1.420f,1.440f,1.460f,1.470f,1.500f,1.520f,1.550f,1.570f,1.600f,1.630f,1.670f,1.690f,1.730f,1.760f,1.790f,1.820f,1.870f,1.900f,1.940f,1.970f,2.030f,2.080f,2.130f,2.170f,2.230f,2.280f,2.340f,2.400f,2.490f,2.550f,2.630f,2.700f,2.810f,2.880f,3.010f,3.090f,3.240f,3.340f,3.500f,3.620f,3.810f,3.950f,4.180f,4.340f,4.620f,8.170f,8.140f,8.150f,8.120f,8.110f,8.100f,8.100f,8.300f,9.040f,9.130f,9.130f,13.030f,18.050f,19.150f,81.910f,20.070f,47.980f,48.040f,48.030f,19.140f,19.180f,19.180f,19.190f,14.550f,19.210f,16.850f,16.840f,7.800f,7.770f,7.770f,7.750f,7.770f,7.760f,7.780f,7.760f,8.320f,8.350f,8.350f,8.090f,7.720f,7.730f,6.430f,6.360f,6.290f,6.260f,6.230f,6.220f,6.160f,5.800f,4.510f,4.410f,4.240f,4.140f,4.000f,3.910f,3.790f,3.680f,3.660f,3.680f,3.680f,3.700f,3.710f,3.730f,3.730f,3.760f,3.770f,3.790f,3.820f,3.850f,3.900f,3.940f,3.980f,3.250f,3.180f,3.140f,3.070f,3.030f,2.970f,2.930f,2.880f,2.850f,2.790f,2.760f,2.710f,2.680f,2.660f,2.670f,2.690f,2.710f,2.720f,2.740f,2.760f,2.770f,2.780f,2.800f,2.170f,2.120f,2.090f,2.060f,2.020f,2.010f,1.990f,1.980f,1.970f,1.960f,1.950f,1.950f,1.940f,1.940f,1.950f,1.940f,1.940f,1.950f,1.930f,1.940f,1.940f,1.940f,1.940f,1.940f,1.950f,1.960f,1.960f,1.980f,1.980f,2.000f,2.010f,2.030f,2.060f,2.090f,2.120f,2.190f,2.560f,2.540f,2.530f,2.520f,2.500f,2.490f,2.470f,2.460f,2.450f,2.440f,2.420f,2.410f,2.400f,2.390f,2.380f,2.370f,2.360f,2.350f,2.340f,2.340f,2.330f,2.320f,2.310f,2.300f,2.290f,2.290f,2.290f,2.280f,2.270f,2.260f,2.260f,2.250f,2.240f,2.240f,2.230f,2.230f,2.220f,2.220f,2.210f,2.210f,2.200f,2.200f,2.190f,2.190f,2.190f,2.180f,2.180f,2.170f,2.170f,2.170f,2.160f,2.160f};
char  SCAN_VALID_2[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

#endif

// ------------------------------------------------------
//				TestICP
// ------------------------------------------------------
void TestICP()
{
  CSimplePointsMap		m1,m2;
  float					runningTime;
  CICP::TReturnInfo		info;
  CICP					ICP;

  // Load scans:
  CObservation2DRangeScan	scan1;
  scan1.aperture = M_PIf;
  scan1.rightToLeft = true;
  scan1.validRange.resize( SCANS_SIZE );
  scan1.scan.resize(SCANS_SIZE);
  ASSERT_( sizeof(SCAN_RANGES_1) == sizeof(float)*SCANS_SIZE );

  memcpy( &scan1.scan[0], SCAN_RANGES_1, sizeof(SCAN_RANGES_1) );
  memcpy( &scan1.validRange[0], SCAN_VALID_1, sizeof(SCAN_VALID_1) );

  CObservation2DRangeScan	scan2 = scan1;
  memcpy( &scan2.scan[0], SCAN_RANGES_2, sizeof(SCAN_RANGES_2) );
  memcpy( &scan2.validRange[0], SCAN_VALID_2, sizeof(SCAN_VALID_2) );

  // Build the points maps from the scans:
  m1.insertObservation( &scan1 );
  m2.insertObservation( &scan2 );

#if MRPT_HAS_PCL
  cout << "Saving map1.pcd and map2.pcd in PCL format...\n";
  m1.savePCDFile("map1.pcd", false);
  m2.savePCDFile("map2.pcd", false);
#endif

  // -----------------------------------------------------
  //	ICP.options.ICP_algorithm = icpLevenbergMarquardt;
  //	ICP.options.ICP_algorithm = icpClassic;
  ICP.options.ICP_algorithm = (TICPAlgorithm)ICP_method;

  ICP.options.maxIterations			= 100;
  ICP.options.thresholdAng			= DEG2RAD(10.0f);
  ICP.options.thresholdDist			= 0.75f;
  ICP.options.ALFA					= 0.5f;
  ICP.options.smallestThresholdDist	= 0.05f;
  ICP.options.doRANSAC = false;

  ICP.options.dumpToConsole();
  // -----------------------------------------------------

  CPose2D		initialPose(0.8f,0.0f,(float)DEG2RAD(0.0f));

  CPosePDFPtr pdf = ICP.Align(
      &m1,
      &m2,
      initialPose,
      &runningTime,
      (void*)&info);

  printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
      runningTime*1000,
      info.nIterations,
      runningTime*1000.0f/info.nIterations,
      info.goodness*100 );

  cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

  CPosePDFGaussian  gPdf;
  gPdf.copyFrom(*pdf);

  cout << "Covariance of estimation: " << endl << gPdf.cov << endl;

  cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
  cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
  cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;

  //cout << "Covariance of estimation (MATLAB format): " << endl << gPdf.cov.inMatlabFormat()  << endl;

  cout << "-> Saving reference map as scan1.txt" << endl;
  m1.save2D_to_text_file("scan1.txt");

  cout << "-> Saving map to align as scan2.txt" << endl;
  m2.save2D_to_text_file("scan2.txt");

  cout << "-> Saving transformed map to align as scan2_trans.txt" << endl;
  CSimplePointsMap m2_trans = m2;
  m2_trans.changeCoordinatesReference( gPdf.mean );
  m2_trans.save2D_to_text_file("scan2_trans.txt");


  cout << "-> Saving MATLAB script for drawing 2D ellipsoid as view_ellip.m" << endl;
  CMatrixFloat COV22 =  CMatrixFloat( CMatrixDouble( gPdf.cov ));
  COV22.setSize(2,2);
  CVectorFloat MEAN2D(2);
  MEAN2D[0] = gPdf.mean.x();
  MEAN2D[1] = gPdf.mean.y();
  {
    ofstream f("view_ellip.m");
    f << math::MATLAB_plotCovariance2D( COV22, MEAN2D, 3.0f);
  }


  // If we have 2D windows, use'em:
#if MRPT_HAS_WXWIDGETS
  if (!skip_window)
  {
    gui::CDisplayWindowPlots	win("ICP results");

    // Reference map:
    vector<float>   map1_xs, map1_ys, map1_zs;
    m1.getAllPoints(map1_xs,map1_ys,map1_zs);
    win.plot( map1_xs, map1_ys, "b.3", "map1");

    // Translated map:
    vector<float>   map2_xs, map2_ys, map2_zs;
    m2_trans.getAllPoints(map2_xs,map2_ys,map2_zs);
    win.plot( map2_xs, map2_ys, "r.3", "map2");

    // Uncertainty
    win.plotEllipse(MEAN2D[0],MEAN2D[1],COV22,3.0,"b2", "cov");

    win.axis(-1,10,-6,6);
    win.axis_equal();

    cout << "Close the window to exit" << endl;
    win.waitForKey();
  }
#endif


}


void transferPclPointCloudXYZToXYPointsMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pc,  CSimplePointsMap*  point_map)
{
  std::vector<float> xs, ys;
  //static int n=0;
  //char buffer[100];
  //snprintf(buffer, sizeof(buffer), "scan_direct_%d.txt", n++);
  //ofstream fos(buffer);

  for (size_t next = 0; next < input_pc->points.size(); ++next)
  {
    pcl::PointXYZ _point = input_pc->points.at(next);
    xs.push_back(_point.x);
    ys.push_back(_point.y);
    //fos<< fixed << setprecision(6) << _point.x << " " << _point.y << endl;
  }
  point_map->setAllPoints(xs, ys);
}

//void transferPclPointCloudToXYPointsMap(VPointCloud::Ptr &input_pc, boost::shared_ptr<CSimplePointsMap> & point_map)
void transferPclPointCloudToXYPointsMap(VPointCloud::Ptr &input_pc,  CSimplePointsMap*  point_map)
{
  std::vector<float> xs, ys;
  //static int n=0;
  //char buffer[100];
  //snprintf(buffer, sizeof(buffer), "scan_direct_%d.txt", n++);
  //ofstream fos(buffer);

  for (size_t next = 0; next < input_pc->points.size(); ++next)
  {
    velodyne_pointcloud::PointXYZIR _point = input_pc->points.at(next);
    xs.push_back(_point.x);
    ys.push_back(_point.y);
    //fos<< fixed << setprecision(6) << _point.x << " " << _point.y << endl;
  }
  point_map->setAllPoints(xs, ys);

}

CSimplePointsMap		g_m1,g_m2;
//CSimplePointsMap		*g_m1,*g_m2;
//boost::shared_ptr<CSimplePointsMap>		g_m1,g_m2;
CICP					ICP;
CPose2D g_icp_result(0.0f,0.0f,(float)DEG2RAD(0.0f));;
double x_icp_g = 0;
double y_icp_g = 0;
double z_icp_g = 0;
double yaw_icp_g = 0;
double roll_icp_g = 0;
double pitch_icp_g = 0;

Eigen::Matrix4f g_lpm_Tm_accum = Eigen::Matrix4f::Identity();


//ofstream os("icp_poses.txt");


void processPointCloud (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  float					runningTime;
  CICP::TReturnInfo		info;
  static bool first_time = true;
  VPointCloud::Ptr cloud(new VPointCloud());
  pcl::fromROSMsg(*cloud_msg , *cloud);
  if (first_time)
  {
    first_time = false;
    //g_m1 = (boost::shared_ptr<CSimplePointsMap>)new CSimplePointsMap();
    //g_m1 = new CSimplePointsMap();
    transferPclPointCloudToXYPointsMap(cloud, &g_m1);
  }
  else
  {
    //delete(g_m1);
    //g_m2 = (boost::shared_ptr<CSimplePointsMap>)new CSimplePointsMap();
    //g_m2 = new CSimplePointsMap();
    transferPclPointCloudToXYPointsMap(cloud, &g_m2);
    CPose2D		initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));

    CPosePDFPtr pdf = ICP.Align(
	&g_m1,
	&g_m2,
	//g_m1.get(),
	//g_m2.get(),
	initialPose,
	&runningTime,
	(void*)&info);

    //return;
    printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
	runningTime*1000,
	info.nIterations,
	runningTime*1000.0f/info.nIterations,
	info.goodness*100 );

    cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

    CPosePDFGaussian  gPdf;
    gPdf.copyFrom(*pdf);
    CPosePDFGaussianInf gInf(gPdf);
    mrpt::math::CMatrixDouble33 information_matrix;
    gInf.getInformationMatrix(information_matrix);


    cout << "Covariance of estimation: " << endl << gPdf.cov << endl;
    cout << "Information of estimation: " << endl << information_matrix << endl;

    cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
    cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
    cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;

    mrpt::math::CVectorDouble icp_result;
    pdf->getMeanVal().getAsVector(icp_result);
    //g_icp_result = g_icp_result + pdf->getMeanVal();

    //g_icp_result.getAsVector(pdf->getMeanVal());
    geometry_msgs::PoseWithCovarianceStamped pose_tobe_published;
    //x_icp_g = x_icp_g + icp_result[0];
    //y_icp_g = y_icp_g + icp_result[1];
    //double x_icp =  cos(yaw_icp_g)*x_icp_g + sin(yaw_icp_g)*y_icp_g + icp_result[0];
    //double y_icp = -sin(yaw_icp_g)*x_icp_g + cos(yaw_icp_g)*y_icp_g + icp_result[1];
    //double x_icp =  cos(yaw_icp_g)*icp_result[0] + sin(yaw_icp_g)*icp_result[1] + x_icp_g;
    //double y_icp = -sin(yaw_icp_g)*icp_result[0] + cos(yaw_icp_g)*icp_result[1] + y_icp_g;
    double x_icp =  cos(-icp_result[2])*icp_result[0] + sin(-icp_result[2])*icp_result[1] ;
    double y_icp = -sin(-icp_result[2])*icp_result[0] + cos(-icp_result[2])*icp_result[1] ;
    x_icp_g =  cos(-yaw_icp_g)*x_icp + sin(-yaw_icp_g)*y_icp + x_icp_g;
    y_icp_g = -sin(-yaw_icp_g)*x_icp + cos(-yaw_icp_g)*y_icp + y_icp_g;
    yaw_icp_g = yaw_icp_g + icp_result[2];
    pose_tobe_published.pose.pose.position.x = x_icp_g;
    pose_tobe_published.pose.pose.position.y = y_icp_g;
    pose_tobe_published.pose.covariance[0] = information_matrix(0,0);
    pose_tobe_published.pose.covariance[1] = information_matrix(0,1);
    pose_tobe_published.pose.covariance[2] = information_matrix(1,1);
    pose_tobe_published.pose.covariance[3] = information_matrix(2,2);
    pose_tobe_published.pose.covariance[4] = information_matrix(0,2);
    pose_tobe_published.pose.covariance[5] = information_matrix(1,2);
    //pose_tobe_published.pose.pose.position.x = icp_result[0];
    //pose_tobe_published.pose.pose.position.y = icp_result[1];
    //pose_tobe_published.pose.pose.orientation.x = icp_result[2];
    pose_tobe_published.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_icp_g);
    pub.publish(pose_tobe_published);
    //cout << icp_result[0] << " " << icp_result[1] << " " << icp_result[2] << endl;
    cout << x_icp_g << " " << y_icp_g << " " << yaw_icp_g << endl;
    //os<< fixed << setprecision(4) << x_icp_g << " " << y_icp_g << " " << yaw_icp_g << " " << icp_result[0] <<" " <<  icp_result[1] <<" " <<  icp_result[2] <<" " <<  information_matrix(0,0) << " " << information_matrix(0,1) << " " << information_matrix(1,1) << " " << information_matrix(2,2) << " " << information_matrix(0,2) << " " << information_matrix(1,2) << " " << endl;
    cout<< fixed << setprecision(4) << x_icp_g << " " << y_icp_g << " " << yaw_icp_g << " " << icp_result[0] <<" " <<  icp_result[1] <<" " <<  icp_result[2] <<" " <<  information_matrix(0,0) << " " << information_matrix(0,1) << " " << information_matrix(1,1) << " " << information_matrix(2,2) << " " << information_matrix(0,2) << " " << information_matrix(1,2) << " " << endl;
    //g_m1 = g_m2;
    g_m1.clear();
    std::vector<float> xs, ys;
    g_m2.getAllPoints(xs, ys);
    g_m1.setAllPoints(xs, ys);

  }
}


VPointCloud::Ptr cloud_for_lpm1;

void processPointCloudUsingLpm(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  static bool _first_time = true;
  SimplePeakFinder * peakC = new SimplePeakFinder(0.3, 0.004);
  BetaGridGenerator *m_descriptorGeneratorB = new BetaGridGenerator(0.02, 0.5, 4, 12);
  CurvatureDetector* m_detectorC = new CurvatureDetector(peakC, 1);
  if (_first_time)
  {
    cloud_for_lpm1 = VPointCloud::Ptr( new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud_for_lpm1);
    _first_time = false;
  }
  else
  {
    VPointCloud::Ptr cloud(new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud);
    DP data(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloud_msg));
    sensor_msgs::PointCloud2 point_cloud_total_ros;//(new PointCloud());
    pcl::toROSMsg(*cloud_for_lpm1, point_cloud_total_ros);
    DP ref(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(point_cloud_total_ros));
    ros::Time tic = ros::Time::now();
    std::cout << "called\n";
    PM::TransformationParameters T = icp(data, ref);
    ros::Time toc = ros::Time::now();

    //{
      using namespace std;
      cout << "Final transformation:" << endl << T << endl;
      cout << (toc -tic).toSec() * 1000 << endl;
      tf::Matrix3x3 Ttf(T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2));
      double roll,pitch,yaw;
      Ttf.getRPY(roll,pitch,yaw);
      cout << " " << roll <<" " <<  pitch <<" " <<  yaw << endl;
      float x = T(0,3); float y = T(1,3);
      double x_icp =  cos(-yaw)*x + sin(-yaw)*y ;
      double y_icp = -sin(-yaw)*x + cos(-yaw)*y ;
      x_icp_g =  cos(-yaw_icp_g)*x_icp + sin(-yaw_icp_g)*y_icp + x_icp_g;
      y_icp_g = -sin(-yaw_icp_g)*x_icp + cos(-yaw_icp_g)*y_icp + y_icp_g;
      yaw_icp_g = yaw_icp_g + yaw;
      //os<< fixed << setprecision(4) << x_icp_g << " " << y_icp_g << " " << yaw_icp_g << " " << x <<" " <<  y <<" " <<  yaw <<" " <<  1.0 << " " << 0. << " " << 1.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << endl;
    //}
    cloud_for_lpm1 = cloud;
  }
}

tf::StampedTransform g_s_transform;
void processPointCloudUsingLpm3d(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  static tf::TransformListener listener;
  static bool _first_time = true;
  SimplePeakFinder * peakC = new SimplePeakFinder(0.3, 0.004);
  BetaGridGenerator *m_descriptorGeneratorB = new BetaGridGenerator(0.02, 0.5, 4, 12);
  CurvatureDetector* m_detectorC = new CurvatureDetector(peakC, 1);
  if (_first_time)
  {
    cloud_for_lpm1 = VPointCloud::Ptr( new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud_for_lpm1);
    _first_time = false;

    try{
      listener.lookupTransform("/velodyne", "/odom",  
	  ros::Time(0), g_s_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  else
  {

    // look up last odom for initial value
    double last_odom_diff_roll, last_odom_diff_pitch, last_odom_diff_yaw;
    double last_odom_diff_x, last_odom_diff_y, last_odom_diff_z;

    tf::StampedTransform s_transform;
    try{
      listener.lookupTransform("/velodyne", "/odom",  
      ros::Time(0), s_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //cout << s_transform;

    last_odom_diff_x = s_transform.getOrigin().x() - g_s_transform.getOrigin().x();
    last_odom_diff_y = s_transform.getOrigin().y() - g_s_transform.getOrigin().y();

    tf::Quaternion s_transform_quat = s_transform.getRotation();
    tf::Quaternion g_s_transform_quat = g_s_transform.getRotation();
    double s_transform_yaw, g_s_transform_yaw;
    double roll, pitch, yaw;
    tf::Matrix3x3(s_transform_quat).getRPY(roll, pitch, s_transform_yaw);
    tf::Matrix3x3(g_s_transform_quat).getRPY(roll, pitch, g_s_transform_yaw);
    last_odom_diff_yaw = s_transform_yaw - g_s_transform_yaw;
    cout << "odom_diff: " << last_odom_diff_x << " " << last_odom_diff_y << " " << last_odom_diff_yaw << endl;


    VPointCloud::Ptr cloud(new VPointCloud());
    pcl::fromROSMsg(*cloud_msg , *cloud);
    DP data(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloud_msg));
    sensor_msgs::PointCloud2 point_cloud_total_ros;//(new PointCloud());
    pcl::toROSMsg(*cloud_for_lpm1, point_cloud_total_ros);
    DP ref(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(point_cloud_total_ros));
    ros::Time tic = ros::Time::now();
    std::cout << "called\n";
    PM::TransformationParameters T_init(4,4);
    T_init << cos(last_odom_diff_yaw), -sin(last_odom_diff_yaw),0,last_odom_diff_x
             ,sin(last_odom_diff_yaw),  cos(last_odom_diff_yaw),0,last_odom_diff_y
             ,0                      , 0                       ,1,0
             ,0                      , 0                       ,0,1;
    //PM::TransformationParameters T = icp(data, ref);
    PM::TransformationParameters T = icp(data, ref, T_init);
    ros::Time toc = ros::Time::now();

    //{
    using namespace std;
    cout << "Final transformation:" << endl << T << endl;
    static float worst_timing = 0;
    float timing = (toc -tic).toSec() * 1000;
    if (timing > worst_timing) worst_timing = timing;
    cout << timing << " ms. worst: "  << worst_timing << " ms" << endl;
    tf::Matrix3x3 Ttf(T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2));
    //double roll,pitch,yaw;
    Ttf.getRPY(roll,pitch,yaw);
    cout << " " << roll <<" " <<  pitch <<" " <<  yaw << endl;
    float x = T(0,3); float y = T(1,3);
    float z = T(2,3);

    Eigen::Matrix4f eT;
    eT << T(0,0),T(0,1),T(0,2),T(0,3),T(1,0),T(1,1),T(1,2),T(1,3),T(2,0),T(2,1),T(2,2),T(2,3),T(3,0), T(3,1), T(3,2), T(3,3);
    //g_lpm_Tm_accum = eT * g_lpm_Tm_accum;
    g_lpm_Tm_accum =  g_lpm_Tm_accum * eT;

    cout << g_lpm_Tm_accum << endl;

    tf::Vector3 origin;
    origin.setValue(g_lpm_Tm_accum(0,3), g_lpm_Tm_accum(1,3), g_lpm_Tm_accum(2,3));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(g_lpm_Tm_accum(0,0), g_lpm_Tm_accum(0,1), g_lpm_Tm_accum(0,2), g_lpm_Tm_accum(1,0), g_lpm_Tm_accum(1,1), g_lpm_Tm_accum(1,2), g_lpm_Tm_accum(2,0), g_lpm_Tm_accum(2,1), g_lpm_Tm_accum(2,2));
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(tfqt);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "lpm"));

    g_s_transform = s_transform;
    // translation part only from lpm
    tf::Vector3 atra;
    Eigen::Affine3f eigen_lpm_accum_transform(g_lpm_Tm_accum);
    tf::vectorEigenToTF(eigen_lpm_accum_transform.translation().cast<double>(), atra);
    //tf::Vector3 lpm_translate_correct(origin - g_s_transform.getOrigin());
    //origin = atra - g_s_transform.getOrigin();

    // rotation part only from lpm
    //Eigen::Quaternionf quat_eig(Eigen::Affine3f(eT).rotation());
    Eigen::Matrix3d odom_accum_rot_eigen;
    tf::matrixTFToEigen(g_s_transform.getBasis(), odom_accum_rot_eigen);
    //Eigen::Quaternionf quat_eig(odom_accum_rot_eigen.inverse().cast<float>() * Eigen::Affine3f(eigen_lpm_accum_transform).rotation());
    Eigen::Quaternionf quat_eig( Eigen::Affine3f(eigen_lpm_accum_transform).rotation()*odom_accum_rot_eigen.inverse().cast<float>()  );
    tf::quaternionEigenToTF(quat_eig.cast<double>(), tfqt);

    // get rotation correction 

    // rotation part side effects from accumulation
    //tf::Vector3 etra(atra);
    tf::Vector3 stra;
    Eigen::Vector3d aT;
    tf::vectorTFToEigen(g_s_transform.getOrigin(), aT);
    //tf::vectorEigenToTF(Eigen::Vector3f(Eigen::Affine3f(eigen_lpm_accum_transform).rotation() * aT).cast<double>(),stra);
    //tf::vectorEigenToTF(odom_accum_rot_eigen.inverse()* Eigen::Affine3f(eigen_lpm_accum_transform).rotation().cast<double>()* aT,stra);
    tf::vectorEigenToTF(Eigen::Affine3f(eigen_lpm_accum_transform).rotation().cast<double>() * odom_accum_rot_eigen.inverse() * aT,stra);
    //origin += atra - stra;
    //origin += - stra;
    origin   = atra - stra;

    tf::Transform lpm_correct_transform;
    lpm_correct_transform.setOrigin(origin);
    tf::Quaternion no_rot_quat(tf::Quaternion::getIdentity());
    lpm_correct_transform.setRotation(tfqt);
    //lpm_correct_transform.setRotation(no_rot_quat);
    br.sendTransform(tf::StampedTransform(lpm_correct_transform, ros::Time::now(), "velodyne", "lpm_correction"));

    //double x_icp =  cos(-yaw)*x + sin(-yaw)*y ;
    //double y_icp = -sin(-yaw)*x + cos(-yaw)*y ;
    //x_icp_g =  cos(-yaw_icp_g)*x_icp + sin(-yaw_icp_g)*y_icp + x_icp_g;
    //y_icp_g = -sin(-yaw_icp_g)*x_icp + cos(-yaw_icp_g)*y_icp + y_icp_g;
    //yaw_icp_g = yaw_icp_g + yaw;

    //geometry_msgs::PoseWithCovarianceStamped pose_tobe_published;
    //pose_tobe_published.pose.pose.position.x = x_icp_g;
    //pose_tobe_published.pose.pose.position.y = y_icp_g;
    //pose_tobe_published.pose.pose.position.z = z_icp_g;

    nav_msgs::Odometry lpm_odom;
    lpm_odom.header.frame_id= "velodyne";
    lpm_odom.child_frame_id = "lpm";
    lpm_odom.header.stamp = ros::Time::now();
    geometry_msgs::Pose lpm_odom_pose;
    tf::poseTFToMsg(transform, lpm_odom_pose);
    lpm_odom.pose.pose = lpm_odom_pose;
    //lpm_odom.pose.pose.position.x = g_lpm_Tm_accum(0,3);
    //lpm_odom.pose.pose.position.y = g_lpm_Tm_accum(1,3);
    //lpm_odom.pose.pose.position.z = g_lpm_Tm_accum(2,3);
    //lpm_odom.pose.pose.orientation.x = tfqt(0);

    pub_odom.publish(lpm_odom);

    //os<< fixed << setprecision(4) << x_icp_g << " " << y_icp_g << " " << yaw_icp_g << " " << x <<" " <<  y <<" " <<  yaw <<" " <<  1.0 << " " << 0. << " " << 1.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << endl;
    //}
    cloud_for_lpm1 = cloud;
  }
}





bool processICPService(icp_test::processICP::Request & req, icp_test::processICP::Response & res)
{
  //std::cout << "hi from ICPService\n";
  //res.result.pose.pose.position.x = 1.334567;
  //res.result.header.frame_id = "semak";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(req.ref , *cloud);
  transferPclPointCloudXYZToXYPointsMap(cloud, &g_m1);
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(req.que , *cloud);
  transferPclPointCloudXYZToXYPointsMap(cloud, &g_m2);
  CPose2D		initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
  float					runningTime;
  CICP::TReturnInfo		info;
  CPosePDFPtr pdf = ICP.Align(
      &g_m1,
      &g_m2,
      //g_m1.get(),
      //g_m2.get(),
      initialPose,
      &runningTime,
      (void*)&info);

  printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
      runningTime*1000,
      info.nIterations,
      runningTime*1000.0f/info.nIterations,
      info.goodness*100 );
  cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

  CPosePDFGaussian  gPdf;
  gPdf.copyFrom(*pdf);
  CPosePDFGaussianInf gInf(gPdf);
  mrpt::math::CMatrixDouble33 information_matrix;
  gInf.getInformationMatrix(information_matrix);

  cout << "Covariance of estimation: " << endl << gPdf.cov << endl;
  cout << "Information of estimation: " << endl << information_matrix << endl;
  cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
  cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
  cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;

  mrpt::math::CVectorDouble icp_result;
  pdf->getMeanVal().getAsVector(icp_result);
  res.result.pose.pose.position.x = icp_result[0];
  res.result.pose.pose.position.y = icp_result[1];
  res.result.pose.pose.position.z = icp_result[2];
  res.result.pose.covariance[0]=information_matrix(0,0);
  res.result.pose.covariance[1]=information_matrix(0,1);
  res.result.pose.covariance[2]=information_matrix(1,1);
  res.result.pose.covariance[3]=information_matrix(2,2);
  res.result.pose.covariance[4]=information_matrix(0,2);
  res.result.pose.covariance[5]=information_matrix(1,2);

  return true;
}

int main(int argc, char **argv)
{
  //try
  //{
  //skip_window = (argc>2);
  //if (argc>1)
  //{
  //ICP_method = atoi(argv[1]);
  //}

  //TestICP();

  //return 0;
  //} catch (exception &e)
  //{
  //cout << "MRPT exception caught: " << e.what() << endl;
  //return -1;
  //}
  //catch (...)
  //{
  //printf("Another exception!!");
  //return -1;
  //}
  ros::init(argc, argv, "icp_lpm_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(20);

  //ros::Subscriber sub = nh.subscribe ("velodyne_packets", 1 , cloud_cb);
  //ros::Subscriber sub = nh.subscribe ("ibeo_points", 1 , processPointCloud);
  //ros::Subscriber sub = nh.subscribe ("ibeo_points_filtered", 1 , processPointCloud);

  //ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , processPointCloudUsingLpm);
  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , processPointCloudUsingLpm3d);

  //ros::Subscriber sub = nh.subscribe ("ibeo_points", 1 , processPointCloudUsingLpm);
  //ros::Subscriber sub = nh.subscribe ("ibeo_points_filtered", 1 , processPointCloudUsingLpm);

  //pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "mrpt_pose2d", 1);
  pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "lpm_pose3d", 1);
  pub_odom = nh.advertise<nav_msgs::Odometry>( "lpm_odometry", 1);

  //ros::ServiceServer service = nh.advertiseService("processICP", processICPService);

  //	ICP.options.ICP_algorithm = icpLevenbergMarquardt;
  //	ICP.options.ICP_algorithm = icpClassic;
  ICP.options.ICP_algorithm = (TICPAlgorithm)ICP_method;

  ICP.options.maxIterations			= 100;
  ICP.options.thresholdAng			= DEG2RAD(10.0f);
  ICP.options.thresholdDist			= 0.75f;
  ICP.options.ALFA					= 0.5f;
  ICP.options.smallestThresholdDist	= 0.05f;
  ICP.options.doRANSAC = false;

  //icp.setDefault();
  ifstream ifs("icp-config.yaml");
  if (!ifs.good()) {std::cout << "can't load icp config file...exiting\n"; return 1;}
  //icp.loadFromYaml(ifs);
  icp.setDefault();

  while (ros::ok()){ros::spinOnce();r.sleep();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
  return 0;
  }


