#ifndef ICP_TEST_ICP_TOOLS_H
#define ICP_TEST_ICP_TOOLS_H

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
#include <icp_test/processICP.h>
#include <tf/transform_broadcaster.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <fstream>

#include "treeoptimizer2.hh"
#include <boost/format.hpp>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;
using namespace PointMatcherSupport;

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

void transferPclPointCloudToXYPointsMap(VPointCloud::Ptr &input_pc,  CSimplePointsMap*  point_map);
void transferPclPointCloudXYZToXYPointsMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pc,  CSimplePointsMap*  point_map);


namespace ICPTools
{
  struct Pose2D
  {
    float x; float y; float yaw;
  };
  struct Point2D
  {
    float x; float y;
  };
  Point2D transformPoint2D(Point2D point, Pose2D pose);
}

#endif
