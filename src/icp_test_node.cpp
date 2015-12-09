#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/keypoints/uniform_sampling.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

// include template implementations to transform a custom point cloud
#include <pcl_ros/impl/transforms.hpp>

/** types of point and cloud to work with */
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;

class ICPProcess
{
  protected:
    ros::Publisher  pub_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //pcl::IterativeClosestPoint<VPoint, VPoint> icp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in  ;//(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out ;//(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud_in  ;//(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud_out ;//(new pcl::PointCloud<pcl::PointXYZ>);

    bool first_time;

  public:
    ICPProcess(ros::NodeHandle & nh)
    {
      pub_ = nh.advertise<sensor_msgs::PointCloud2>( "filtered_points", 1);
      first_time = true;

      cloud_in  = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);


    }
    ~ICPProcess()
    {
      //delete cloud_out;
      //delete cloud_in;

    }

    void pcCallback (const sensor_msgs::PointCloud2ConstPtr &scanMsg)
    {
      //VPointCloud::Ptr orig_cloud(new VPointCloud());
      pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::fromROSMsg(*scanMsg , *orig_cloud);
      pcl::fromROSMsg(*scanMsg , *cloud_out);
      //outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      //outMsg->header.frame_id = "base_imu";
      cloud_out->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      cloud_out->header.frame_id = "base_imu";
      orig_cloud->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      orig_cloud->header.frame_id = "base_imu";



      pcl::PointCloud<int> indices_out;
      pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
      uniform_sampling.setInputCloud (orig_cloud);
      uniform_sampling.setRadiusSearch (0.05f);
      //uniform_sampling.filter (*model_keypoints);
      uniform_sampling.compute (indices_out);
      std::cout << "Model total points: " << orig_cloud->size () << "; Selected Keypoints: " << indices_out.size () << std::endl;
      pcl::copyPointCloud(*orig_cloud, indices_out.points, *cloud_out);

      //pcl::PointCloud<int> indices_out;
      //uniform_sampling.setInputCloud (cloud_in);
      //uniform_sampling.compute (*indices_out);
      //pcl::copyPointCloud(*orig_cloud, indices_out.points, *cloud_out);

      if (!first_time)
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	icp.align(*Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	  icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	pub_.publish(Final);
      }

      //pub_.publish(outMsg);
      //pub_.publish(cloud_out);
      *cloud_in = *cloud_out;
      //cloud_in = cloud_out;

      first_time = false;

    }



};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp_test_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(10);

  ICPProcess icp_process(nh);

  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1 , &ICPProcess::pcCallback, &icp_process);


  while (ros::ok()){ros::spinOnce();}//ROS_INFO_STREAM("Hello, world!");r.sleep();}

  return (0);
}
