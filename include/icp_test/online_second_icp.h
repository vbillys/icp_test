#ifndef ICP_TEST_ONLINE_SECOND_ICP
#define ICP_TEST_ONLINE_SECOND_ICP

#include "icp_test/icp_tools.h"
#include <tf/transform_broadcaster.h>

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
    CSimplePointsMap	m_global_map;
    CSimplePointsMap	m_local_map;
    VPointCloud::Ptr m_local_cloud;

    ICPTools::Pose2D m_repose;
    ICPTools::Pose2D m_odom;
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


  public:
    SecondICP();

};

#endif

