#ifndef ICP_TEST_ONLINE_ICP_NODE_H
#define ICP_TEST_ONLINE_ICP_NODE_H

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>
//#include <math.h>

#define SCAN_MATCHING_MAX_NO_OF_LOCAL_MAPS 10
#define SCAN_MATCHING_DISTANCE_BETWEEN_CONSECUTIVE_LOCAL_MAPS 2.5

class ScanMatchingI2R
{

  private:

    struct LocalMap
    {
      CSimplePointsMap points_set;
      ICPTools::Pose2D global_pose;
      
    };
    vector<LocalMap> m_local_maps;

    int  m_ICP_method;
    ros::Subscriber m_sub;
    ros::Publisher m_pub_pose;
    ros::Publisher m_pub_localmap;
    ros::NodeHandle m_nh;
    CICP					m_ICP;
    double m_x_icp_g;
    double m_y_icp_g;
    double m_yaw_icp_g;
    CSimplePointsMap		m_m1,m_m2, m_filtered_map;
    CPose2D	m_initialPose;
    bool   m_first_time;
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

    sensor_msgs::PointCloud2ConstPtr m_cloud_msg;

    tf::TransformBroadcaster m_t_br;
    tf::Transform m_transform;

    void ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void SetDefaultICPOptions();
    void ResetICPAccumulatedPoseAndClearState();
    void DoICP();
    void ComputeInformationMatrix();
    void ComputeAccumulatedPose();
    bool PushToLocalMapUsingGlobalPose(const CSimplePointsMap & input_map);
    void ComputeStraightDistanceTravelledFromMarkedPose();
    void MarkPose();
    void FilterPointCloud(VPointCloud::Ptr & input_cloud);
    void GetAllPointsFromLocalMap(vector<ICPTools::Point2D> & points);
    void ToPclFromLocalMapFromPoints2D(vector<ICPTools::Point2D> & points, VPointCloud::Ptr & output_cloud);
    void StorePointerToRosMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    template<typename T> void StampPclMsgWithOrigRosMsg(T & pcl_msg);

    const static int m_c_max_no_of_local_maps = SCAN_MATCHING_MAX_NO_OF_LOCAL_MAPS;
    const static float m_c_distance_between_local_maps = SCAN_MATCHING_DISTANCE_BETWEEN_CONSECUTIVE_LOCAL_MAPS;

  public:
    ScanMatchingI2R();
    void printOutStatus();
    void publishPose();
    void TransferScanDataToRefData();
    void publishLocalMap();
};

#endif
