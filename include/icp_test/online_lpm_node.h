#ifndef ICP_TEST_ONLINE_LPM_NODE_H
#define ICP_TEST_ONLINE_LPM_NODE_H

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <nav_msgs/Odometry.h>
//#include <math.h>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>

#define SCAN_MATCHING_MAX_NO_OF_LOCAL_MAPS 15 //10 // 15 //10
#define SCAN_MATCHING_DISTANCE_BETWEEN_CONSECUTIVE_LOCAL_MAPS 3.5 //4.5 //3.5 //2.5
#define SCAN_MATCHING_DISTANCE_BETWEEN_CONSECUTIVE_ODOM_MAPS 0.15 //4.5 //3.5 //2.5
#define SCAN_MATCHING_DISTANCE_LOAM_UPDATE .5 //.3//1 //.3 //1. //2. //.3 //2. //0.3
#define SCAN_MATCHING_WORST_TIME_BETWEEN_LOAMS 0.5 //.25 //1 //.25 //2.5 //.6 //.25 //2. //.25 //2. //0.25

class ScanMatching3D
{

  private:

    struct LocalMap
    {
      CSimplePointsMap points_set;
      ICPTools::Pose2D global_pose;

    };
    vector<LocalMap> m_local_maps;

    struct LocalMap3D
    {
      //VPointCloud points_set;
      sensor_msgs::PointCloud2ConstPtr points_set;
      Eigen::Matrix4f global_pose;
      
    };
    vector<LocalMap3D> m_local_3d_maps;

    enum OdomSearchMode {none, first_first, first_second, second_first, second_second};

    int  m_ICP_method;
    ros::Subscriber m_sub;
    ros::Publisher m_pub_pose;
    ros::Publisher m_pub_localmap;
    ros::Publisher m_pub_lpm_odom;
    ros::NodeHandle m_nh;
    CICP					m_ICP;
    double m_x_icp_g;
    double m_y_icp_g;
    double m_yaw_icp_g;
    CSimplePointsMap		m_m1,m_m2, m_filtered_map;
    CPose2D	m_initialPose;
    bool   m_first_time;
    OdomSearchMode  m_odom_local_to_32;
    float	m_runningTime;
    CICP::TReturnInfo	m_info;
    CPosePDFPtr m_pdf;
    mrpt::math::CMatrixDouble33 m_information_matrix;
    CPosePDFGaussian  m_gPdf;
    mrpt::math::CVectorDouble m_icp_result;
    ICPTools::Pose2D m_pose;

    ICPTools::Pose2D m_marked_pose;
    float m_travelled_dist;
    float m_next_capture_dist;

    std::ifstream m_g_cfg_ifs;
    PM::DataPointsFilters m_g_dpf;
    boost::scoped_ptr<PM::DataPointsFilters> m_p_g_dpf;

    sensor_msgs::PointCloud2ConstPtr m_cloud_msg;
    sensor_msgs::PointCloud2ConstPtr m_m1_3d, m_m2_3d;

    tf::TransformBroadcaster m_t_br;
    tf::Transform m_transform;

    PM::ICP m_icp_3d;
    PM::TransformationParameters m_T_lpm;
    Eigen::Matrix4f m_g_lpm_Tm_accum; 
    bool m_is_valid;
    tf::TransformListener m_listener;
    tf::StampedTransform m_g_s_transform;
    tf::StampedTransform m_l_s_transform;
    ros::Time m_last_loam_start;
    tf::Transform m_lpm_correct_transform;

    float m_worst_timing;
    string m_world_frame;

    void ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessPointCloud3D32(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessPointCloud3D(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void SetDefaultICPOptions();
    void ResetICPAccumulatedPoseAndClearState();
    void DoICP();
    void DoICP3D(ICPTools::Pose2D & init);
    void DoICP3DWithLocalMap(ICPTools::Pose2D & init);
    void ComputeInformationMatrix();
    void ComputeAccumulatedPose();
    bool PushToLocalMapUsingGlobalPose(const CSimplePointsMap & input_map);
    void PushThisCloudTo3DLocalMap();
    void ComputeStraightDistanceTravelledFromMarkedPose();
    void MarkPose();
    void FilterPointCloud(VPointCloud::Ptr & input_cloud);
    void GetAllPointsFromLocalMap(vector<ICPTools::Point2D> & points);
    void ToPclFromLocalMapFromPoints2D(vector<ICPTools::Point2D> & points, VPointCloud::Ptr & output_cloud);
    void ToPclFromLocalMapRos3D(VPointCloud::Ptr pcl_cloud);
    void StorePointerToRosMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    template<typename T> void StampPclMsgWithOrigRosMsg(T & pcl_msg);
    template<typename T> void StampPclMsgWithLpm(T & pcl_msg);
    Eigen::Matrix4f GetTFAsEigen(string name);


    //void GetTF(tf::StampedTransform & otf);
    bool GetTF(tf::StampedTransform & otf, string name);
    ICPTools::Pose2D GetOdomDiff(tf::StampedTransform &before, tf::StampedTransform &after);

    const static int m_c_max_no_of_local_maps = SCAN_MATCHING_MAX_NO_OF_LOCAL_MAPS;
    const static float m_c_distance_between_local_maps = SCAN_MATCHING_DISTANCE_BETWEEN_CONSECUTIVE_LOCAL_MAPS;
    const static float m_c_distance_between_consecutive_odom_maps = SCAN_MATCHING_DISTANCE_BETWEEN_CONSECUTIVE_ODOM_MAPS;
    const static float m_c_distance_loam_update = SCAN_MATCHING_DISTANCE_LOAM_UPDATE;
    const static float m_c_worst_time_between_loams = SCAN_MATCHING_WORST_TIME_BETWEEN_LOAMS;

  public:
    ScanMatching3D(string world_frame);
    void printOutStatus();
    void publishPose();
    void publishPose3D();
    void TransferScanDataToRefData();
    void publishLocalMap();
    void publishLocalMap3D();
    bool isValid();
};

#endif
