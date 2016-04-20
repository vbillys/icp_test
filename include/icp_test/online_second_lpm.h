#ifndef ICP_TEST_ONLINE_ICP_NODE_H
#define ICP_TEST_ONLINE_ICP_NODE_H

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <nav_msgs/Odometry.h>
//#include <math.h>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <nav_msgs/Odometry.h>
//#include <math.h>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <nav_msgs/Odometry.h>
//#include <math.h>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>


#define SCAN_MATCHING_DISTANCE_GLOBAL_LOAM_UPDATE 2.5 //.3//1 //.3 //1. //2. //.3 //2. //0.3
#define SCAN_MATCHING_WORST_TIME_BETWEEN_GLOBAL_LOAMS 3.5 //0.5 //.25 //1 //.25 //2.5 //.6 //.25 //2. //.25 //2. //0.25

class SecondLPM
{
  public:
    SecondLPM(string world_frame);

  private:

    string m_world_frame;
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_globalmap, m_sub_lpm, m_sub_localmap;
    sensor_msgs::PointCloud2ConstPtr m_global_map_msg;
    tf::TransformBroadcaster m_t_br;
    tf::TransformListener m_listener;

    bool m_global_map_received;

    void ProcessLocalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessGlobalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessLpmOdometry(const nav_msgs::OdometryPtr& odom_msg);
    bool GetTF(tf::StampedTransform & otf, string name);

    const static float m_c_distance_global_loam_update = SCAN_MATCHING_DISTANCE_GLOBAL_LOAM_UPDATE;
    const static float m_c_worst_time_between_global_loams = SCAN_MATCHING_WORST_TIME_BETWEEN_GLOBAL_LOAMS;

};

#endif
