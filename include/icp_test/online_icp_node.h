#ifndef ICP_TEST_ONLINE_ICP_NODE_H
#define ICP_TEST_ONLINE_ICP_NODE_H

#include "icp_test/icp_tools.h"

class ScanMatchingI2R
{

  private:
    int  m_ICP_method;
    ros::Subscriber m_sub;
    ros::Publisher m_pub_pose;
    ros::Publisher m_pub_localmap;
    ros::NodeHandle m_nh;
    CICP					m_ICP;
    double m_x_icp_g;
    double m_y_icp_g;
    double m_yaw_icp_g;
    CSimplePointsMap		m_m1,m_m2;
    bool   m_first_time;

    void ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void SetDefaultICPOptions();
    void ResetICPAccumulatedPoseAndClearState();

  public:
    ScanMatchingI2R();
};

#endif
