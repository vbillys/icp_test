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
#include <std_msgs/Int32.h>
#include "file_writer.h"
#include "ecl/threads.hpp"

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
ros::Publisher g_pub_synctime;
tf::Pose g_velo1_pose, g_velo2_pose, g_velo3_pose;
bool g_waiting_for_tf;

void callback(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2, const sensor_msgs::PointCloud2ConstPtr& pc3)
{
  if (g_waiting_for_tf) {
    ROS_WARN("Due to unknown transfrom cannot combine");
    return;
  }

  sensor_msgs::PointCloud2 combined;
  combined.header.stamp = ros::Time::now();
  g_pub_synctime.publish(std_msgs::Int32());

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
  pcl::toROSMsg(latest_pcl_pc1, combined);
  ROS_INFO("combined: %.6f 1st: %.6f 2nd: %.6f 3nd: %.6f", combined.header.stamp.toSec(), pc1->header.stamp.toSec(), pc2->header.stamp.toSec(), pc3->header.stamp.toSec());
  ROS_WARN("Diff 1st: %.6f 2nd: %.6f 3nd: %.6f", combined.header.stamp.toSec()- pc1->header.stamp.toSec(), combined.header.stamp.toSec()-pc2->header.stamp.toSec(), combined.header.stamp.toSec()-pc3->header.stamp.toSec());
  combined.header.frame_id = g_settings.combined_frame_name;
  g_pub_combined.publish(combined);
}

ecl::Mutex g_pc1_mutex, g_pc2_mutex, g_pc3_mutex;
ecl::Mutex g_transform_mutex;
PCLPointCloudXYI g_latest_pcl_pc1, g_latest_pcl_pc2, g_latest_pcl_pc3;
double g_latest_time_pc1, g_latest_time_pc2, g_latest_time_pc3;
bool g_update_flag_pc1, g_update_flag_pc2, g_update_flag_pc3;
bool g_transform_flag;
void HandlePC1(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  g_pc1_mutex.lock();
  g_transform_mutex.lock();
  pcl::fromROSMsg(*pc_msg, g_latest_pcl_pc1);
  g_latest_time_pc1 = pc_msg->header.stamp.toSec();
  g_update_flag_pc1 = true;
  g_pc1_mutex.unlock();
  g_transform_mutex.unlock();
}

void HandlePC2(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  g_pc2_mutex.lock();
  g_transform_mutex.lock();
  pcl::fromROSMsg(*pc_msg, g_latest_pcl_pc2);
  g_latest_time_pc2 = pc_msg->header.stamp.toSec();
  g_update_flag_pc2 = true;
  g_pc2_mutex.unlock();
  g_transform_mutex.unlock();
}

void HandlePC3(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  g_pc3_mutex.lock();
  g_transform_mutex.lock();
  pcl::fromROSMsg(*pc_msg, g_latest_pcl_pc3);
  g_latest_time_pc3 = pc_msg->header.stamp.toSec();
  g_update_flag_pc3 = true;
  g_pc3_mutex.unlock();
  g_transform_mutex.unlock();
}

void HandleSyncTimer(const ros::TimerEvent& event)
{
  g_pc1_mutex.lock();
  g_pc2_mutex.lock();
  g_pc3_mutex.lock();
  g_transform_mutex.lock();
  if (g_update_flag_pc1 && g_update_flag_pc2 && g_update_flag_pc3) {
    g_pub_synctime.publish(std_msgs::Int32());
    g_update_flag_pc3 = false;
    g_update_flag_pc2 = false;
    g_update_flag_pc1 = false;
    g_transform_flag  = true;
  }
  g_pc1_mutex.unlock();
  g_pc2_mutex.unlock();
  g_pc3_mutex.unlock();
  g_transform_mutex.unlock();
}

void HandleTransformTimer(const ros::TimerEvent& event)
{
  if (!g_transform_flag) {
    return;
  }
  g_transform_mutex.lock();

  sensor_msgs::PointCloud2 combined;
  combined.header.stamp = ros::Time::now();

  PCLPointCloudXYI latest_pcl_pc1, latest_pcl_pc2, latest_pcl_pc3;

  pcl_ros::transformPointCloud(g_latest_pcl_pc1, latest_pcl_pc1, g_velo1_pose);
  pcl_ros::transformPointCloud(g_latest_pcl_pc2, latest_pcl_pc2, g_velo2_pose);
  pcl_ros::transformPointCloud(g_latest_pcl_pc3, latest_pcl_pc3, g_velo3_pose);
  // combine
  latest_pcl_pc1 += latest_pcl_pc2;
  latest_pcl_pc1 += latest_pcl_pc3;
  pcl::toROSMsg(latest_pcl_pc1, combined);
  g_pub_combined.publish(combined);

  g_transform_flag  = false;
  g_transform_mutex.unlock();
}

int main(int argc, char *argv[])
{
  g_update_flag_pc1 = false;
  g_update_flag_pc2 = false;
  g_update_flag_pc3 = false;
  g_transform_flag  = false;

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
  g_pub_synctime
    = nh.advertise<std_msgs::Int32 > ("merged", 1);

  //message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_1_sub(nh, g_settings.velo1_topic_name, 5);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_2_sub(nh, g_settings.velo2_topic_name, 5);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_3_sub(nh, g_settings.velo3_topic_name, 5);
  ros::Subscriber pointcloud_1_sub = nh.subscribe(g_settings.velo1_topic_name, 5, HandlePC1);
  ros::Subscriber pointcloud_2_sub = nh.subscribe(g_settings.velo2_topic_name, 5, HandlePC2);
  ros::Subscriber pointcloud_3_sub = nh.subscribe(g_settings.velo3_topic_name, 5, HandlePC3);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), pointcloud_1_sub, pointcloud_2_sub, pointcloud_3_sub);
  //sync.setInterMessageLowerBound(0,ros::Duration(0.05));
  //sync.setInterMessageLowerBound(1,ros::Duration(0.05));
  //sync.setInterMessageLowerBound(2,ros::Duration(0.05));
  //sync.setMaxIntervalDuration(ros::Duration(0.025));
  //sync.registerCallback(boost::bind(&callback, _1, _2, _3));

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

  // do our own horse powered timestamp checks
  // 
  ros::Timer sync_timer = nh.createTimer(ros::Duration(0.005), HandleSyncTimer);
  ros::Timer transform_timer = nh.createTimer(ros::Duration(0.05), HandleTransformTimer);


  ros::spin();
  return 0;
}
/*===========================================================================*/
/*===============================[   ]===============================*/
/*===========================================================================*/
 
 

