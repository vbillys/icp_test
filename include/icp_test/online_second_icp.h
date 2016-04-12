#ifndef ICP_TEST_ONLINE_SECOND_ICP
#define ICP_TEST_ONLINE_SECOND_ICP

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>

#define SECOND_ICP_GLOBALMAP_CROP 2500 //3600 //2500 //900 //2500
#define SECOND_ICP_LOCALMAP_CROP  45 //60 //45 //25 //45
#define SECOND_ICP_MAX_ITER 500 //1000
#define SECOND_ICP_ALPHA .5 //.8

class SecondICP
{
  private:
    ros::Subscriber m_sub_localmap;
    ros::Subscriber m_sub_globalmap;
    ros::Subscriber m_sub_pose;
    ros::Publisher m_pub_repose;
    ros::Publisher m_pub_debug_global_icp;
    ros::Publisher m_pub_debug_local_crop;
    ros::NodeHandle m_nh;

    CICP					m_ICP;
    int  m_ICP_method;
    float	m_runningTime;
    CICP::TReturnInfo	m_info;
    CPosePDFPtr m_pdf;
    mrpt::math::CMatrixDouble33 m_information_matrix;
    CPosePDFGaussian  m_gPdf;
    mrpt::math::CVectorDouble m_icp_result;

    CSimplePointsMap	m_global_map, m_current_global_map;
    CSimplePointsMap	m_local_map;
    VPointCloud::Ptr m_local_cloud;

    ICPTools::Pose2D m_predicted_pose;
    ICPTools::Pose2D m_repose;
    ICPTools::Pose2D m_odom;
    ICPTools::Pose2D m_odom_not_cached;
    ICPTools::Pose2D m_last_odom;

    tf::TransformBroadcaster m_t_br;
    tf::Transform m_retransform;


    vector<size_t> m_out_idx; vector<float> m_out_dist;
    vector<std::pair< size_t, float > > m_tree_pair;

    void SetDefaultICPOptions();
    void ResetAndClearState();
    void ProcessLocalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void ProcessOdomPose(const geometry_msgs::PoseWithCovarianceStampedPtr& pose_msg);
    void ProcessGlobalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void DoKDTreeSearch();
    void PublishDebugKDTreeSearch();
    void CropLocalCloud();
    void PublishDebugCropLocalCloud();
    void DoICP();
    void DoICPWithWholeGlobalMap();
    void UpdateRepose();
    void ProposeNewVehicleLocation();

    const static float m_c_globalmap_crop = SECOND_ICP_GLOBALMAP_CROP;
    const static float m_c_localmap_crop  = SECOND_ICP_LOCALMAP_CROP;
    const static int m_c_max_iter = SECOND_ICP_MAX_ITER;
    const static float m_c_icp_alpha = SECOND_ICP_ALPHA;

  public:
    SecondICP();
    void printOutStatus();

};

#endif

