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
    void publishNecessaries();
    bool isValid();

  private:

    string m_world_frame;
    ros::NodeHandle m_nh;
    ros::Publisher m_pub_debug_global_map;
    ros::Publisher m_pub_debug_local_map;
    ros::Subscriber m_sub_globalmap, m_sub_lpm, m_sub_localmap, m_sub_localmap_last_lpm;
    sensor_msgs::PointCloud2ConstPtr m_global_map_msg;
    VPointCloud m_global_map_cloud;
    VPointCloud::Ptr m_global_map_filtered;
    tf::TransformBroadcaster m_t_br;
    tf::TransformListener m_listener;
    Eigen::Affine3d m_eigen_lpm_transform;
    Eigen::Affine3f m_eigen_affine_last_mapped_lpm;
    geometry_msgs::Pose m_last_lpm_mapped_pose_msg;


    PM::DataPointsFilters m_g_dpf;
    std::ifstream m_g_cfg_ifs;
    boost::scoped_ptr<PM::DataPointsFilters> m_p_g_dpf;
    PM::ICP m_icp_3d;
    float m_worst_timing;
    PM::TransformationParameters m_T_lpm;
    Eigen::Matrix4f m_g_lpm_Tm_accum; 
    DP m_dp_global, m_dp_global_transformed;

    bool m_global_map_received;
    bool m_is_valid;

    void ProcessLocalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessGlobalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessLpmOdometry(const nav_msgs::OdometryPtr& odom_msg);
    bool GetTF(tf::StampedTransform & otf, string name);
    Eigen::Matrix4f GetTFAsEigen(string name);
    void TransformCloud(Eigen::Affine3f affine, VPointCloud & cloud, VPointCloud::Ptr& cloud_transformed);
    void GetEigenLastLpmPose(Eigen::Affine3f & affine);
    void ProcessLastLpmPoseMapped(const geometry_msgs::PosePtr& pose_msg);
    template<typename T> void StampPclMsg(T & pcl_msg);
    void FilterPointCloud(VPointCloud & input_cloud, VPointCloud::Ptr  output_cloud);
    void DoGlobalICP(sensor_msgs::PointCloud2 & iref, sensor_msgs::PointCloud2 & idata, Eigen::Matrix4f & eigen_init);

    const static float m_c_distance_global_loam_update = SCAN_MATCHING_DISTANCE_GLOBAL_LOAM_UPDATE;
    const static float m_c_worst_time_between_global_loams = SCAN_MATCHING_WORST_TIME_BETWEEN_GLOBAL_LOAMS;

};

#endif
