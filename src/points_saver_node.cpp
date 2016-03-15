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
#include <cstdio>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>

#include "treeoptimizer2.hh"


/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

ros::Publisher  pub;

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

//void transferPclPointCloudToXYPointsMap(VPointCloud::Ptr &input_pc,  CSimplePointsMap*  point_map)
void transferPclPointCloudToXYPointsMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pc,  CSimplePointsMap*  point_map)
{
  std::vector<float> xs, ys;
  for (size_t next = 0; next < input_pc->points.size(); ++next)
  {
    //velodyne_pointcloud::PointXYZIR _point = input_pc->points.at(next);
    pcl::PointXYZ _point = input_pc->points.at(next);
    xs.push_back(_point.x);
    ys.push_back(_point.y);
  }
  point_map->setAllPoints(xs, ys);

}

CSimplePointsMap		g_m1,g_m2;
void savePointCloud (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  static int n=0;
  //VPointCloud::Ptr cloud(new VPointCloud());
  //PointXYZ::Ptr cloud(new PointXYZ());
  //pcl::PointCloud<pcl::PointXYZ> _cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(_cloud); //(&(new pcl::PointCloud<pcl::PointXYZ>));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(_cloud); //(&(new pcl::PointCloud<pcl::PointXYZ>));
  //cloud = _cloud.makeShared();

  pcl::fromROSMsg(*cloud_msg , *cloud);
  transferPclPointCloudToXYPointsMap(cloud, &g_m1);
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "scan_%d.txt", n++);
  bool ok_is_saved = g_m1.save2D_to_text_file(std::string(buffer));
  std::cout << buffer << " " << ok_is_saved << std::endl;

}

void savePointCloudFiltered (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  static int n=0;
  //VPointCloud::Ptr cloud(new VPointCloud());
  //PointXYZ::Ptr cloud(new PointXYZ());
  //pcl::PointCloud<pcl::PointXYZ> _cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(_cloud); //(&(new pcl::PointCloud<pcl::PointXYZ>));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(_cloud); //(&(new pcl::PointCloud<pcl::PointXYZ>));
  //cloud = _cloud.makeShared();

  pcl::fromROSMsg(*cloud_msg , *cloud);
  transferPclPointCloudToXYPointsMap(cloud, &g_m2);
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "scan_filtered_%d.txt", n++);
  bool ok_is_saved = g_m2.save2D_to_text_file(std::string(buffer));
  std::cout << buffer << " " << ok_is_saved << std::endl;

}

void processICPResult (const geometry_msgs::PoseWithCovarianceStampedPtr& pose_msg)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_saver_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(20);
  //ros::Subscriber sub = nh.subscribe ("ibeo_points_filtered", 1 , savePointCloud);
  ros::Subscriber sub = nh.subscribe ("ibeo_points_icp_only_current", 1 , savePointCloud);
  ros::Subscriber sub2= nh.subscribe ("ibeo_points_icp_only_current_filtered", 1 , savePointCloudFiltered);
  //ros::Subscriber sub2= nh.subscribe ("mrpt_pose2d", 1 , processICPResult);
  while (ros::ok()){ros::spinOnce();r.sleep();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}
  return 0;
}
