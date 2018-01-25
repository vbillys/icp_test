/**@file	/home/zy/Documents/workspace/icp_test/src/velosync.cpp
 * @author	zy
 * @version	800
 * @date
 * 	Created:	15th Sep 2017
 * 	Last Update:	15th Sep 2017
 */

/*===========================================================================*/
/*===============================[  ]===============================*/
/*===========================================================================*/
 
#include "tf/transform_listener.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloudXYI;

struct Settings
{
  std::string base1_frame_name;
  std::string base2_frame_name;
  std::string base3_frame_name;
  std::string velo1_frame_name;
  std::string velo2_frame_name;
  std::string velo3_frame_name;
  std::string velo1_topic_name;
  std::string velo2_topic_name;
  std::string velo3_topic_name;
  std::string combined_frame_name;
}g_settings;

ros::Publisher g_pub_combined;
tf::Pose g_velo1_pose, g_velo2_pose, g_velo3_pose;
bool g_waiting_for_tf;

void callback(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2, const sensor_msgs::PointCloud2ConstPtr& pc3)
{
  if (g_waiting_for_tf) {
    ROS_WARN("Due to unknown transfrom cannot combine");
    return;
  }
  PCLPointCloudXYI latest_pcl_pc1, latest_pcl_pc2, latest_pcl_pc3;
  pcl::fromROSMsg(*pc1, latest_pcl_pc1);
  pcl::fromROSMsg(*pc2, latest_pcl_pc2);
  pcl::fromROSMsg(*pc3, latest_pcl_pc3);
  // we see from base_link instead
  pcl_ros::transformPointCloud(latest_pcl_pc1, latest_pcl_pc1, g_velo1_pose);
  pcl_ros::transformPointCloud(latest_pcl_pc2, latest_pcl_pc2, g_velo2_pose);
  pcl_ros::transformPointCloud(latest_pcl_pc3, latest_pcl_pc3, g_velo3_pose);
  // combine
  latest_pcl_pc1 += latest_pcl_pc2;
  latest_pcl_pc1 += latest_pcl_pc3;
  sensor_msgs::PointCloud2 combined;
  pcl::toROSMsg(latest_pcl_pc1, combined);
  combined.header.stamp = ros::Time::now();
  combined.header.frame_id = g_settings.combined_frame_name;
  g_pub_combined.publish(combined);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "velosync");

  ros::NodeHandle nh;

  tf::TransformListener tf_listener;

  // handle parameters
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("base1_frame" , g_settings.base1_frame_name , "base_link");
  pnh.param<std::string>("base2_frame" , g_settings.base2_frame_name , "base_link");
  pnh.param<std::string>("base3_frame" , g_settings.base3_frame_name , "base_link");
  pnh.param<std::string>("velo1_frame", g_settings.velo1_frame_name, "velodyne_below");
  pnh.param<std::string>("velo2_frame", g_settings.velo2_frame_name, "velodyne_upper");
  pnh.param<std::string>("velo3_frame", g_settings.velo3_frame_name, "velodyne_upper");
  pnh.param<std::string>("velo1_topic", g_settings.velo1_topic_name, "velodyne1/velodyne_points");
  pnh.param<std::string>("velo2_topic", g_settings.velo2_topic_name, "velodyne2/velodyne_points");
  pnh.param<std::string>("velo3_topic", g_settings.velo3_topic_name, "velodyne3/velodyne_points");
  pnh.param<std::string>("combined_frame", g_settings.combined_frame_name, "combined");


  g_pub_combined = nh.advertise<sensor_msgs::PointCloud2 > ("combined_cloud", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_1_sub(nh, g_settings.velo1_topic_name, 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_2_sub(nh, g_settings.velo2_topic_name, 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_3_sub(nh, g_settings.velo3_topic_name, 5);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), pointcloud_1_sub, pointcloud_2_sub, pointcloud_3_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::Rate loop_rate(15);
  g_waiting_for_tf = true;
  tf::Stamped<tf::Pose> velo1_tf, velo2_tf, velo3_tf;
  // we wait until tf come
  do {

    ros::spinOnce();

    tf::Stamped<tf::Pose> ident1 (tf::Transform(tf::createIdentityQuaternion(),
				 tf::Vector3(0,0,0)),
				 ros::Time(), g_settings.velo1_frame_name);
    tf::Stamped<tf::Pose> ident2 (tf::Transform(tf::createIdentityQuaternion(),
				 tf::Vector3(0,0,0)),
				 ros::Time(), g_settings.velo2_frame_name);
    tf::Stamped<tf::Pose> ident3 (tf::Transform(tf::createIdentityQuaternion(),
	  tf::Vector3(0,0,0)),
	ros::Time(), g_settings.velo3_frame_name);
    
    try {
      tf_listener.transformPose(g_settings.base1_frame_name,ident1, velo1_tf);
      tf_listener.transformPose(g_settings.base2_frame_name,ident2, velo2_tf);
      tf_listener.transformPose(g_settings.base3_frame_name,ident3, velo3_tf);
    }catch(tf::TransformException & e) {
      ROS_ERROR_DELAYED_THROTTLE(3,"Can't transform to base");
      continue;
    }

    // save aquired tfs
    g_velo1_pose = velo1_tf;
    g_velo2_pose = velo2_tf;
    g_velo3_pose = velo3_tf;
    g_waiting_for_tf = false;
    
    
  } while (g_waiting_for_tf && ros::ok());
  ROS_INFO("tf locked :)");

  ros::spin();
  return 0;
}
/*===========================================================================*/
/*===============================[   ]===============================*/
/*===========================================================================*/
 
 

