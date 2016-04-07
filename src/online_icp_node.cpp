#include "icp_test/online_icp_node.h"

ScanMatchingI2R::ScanMatchingI2R()
{

  m_ICP_method = (int) icpClassic;
  m_sub = m_nh.subscribe ("ibeo_points", 1 , &ScanMatchingI2R::ProcessPointCloud, this);
  m_pub_pose = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "icp_pose2d", 10);
  m_pub_localmap = m_nh.advertise<sensor_msgs::PointCloud2>( "icp_localmap", 2);
  SetDefaultICPOptions();
  ResetICPAccumulatedPoseAndClearState();
}

void ScanMatchingI2R::SetDefaultICPOptions()
{
  m_ICP.options.ICP_algorithm = (TICPAlgorithm)m_ICP_method;
  m_ICP.options.maxIterations			= 100;
  m_ICP.options.thresholdAng			= DEG2RAD(10.0f);
  m_ICP.options.thresholdDist			= 0.75f;
  m_ICP.options.ALFA					= 0.5f;
  m_ICP.options.smallestThresholdDist	= 0.05f;
  m_ICP.options.doRANSAC = false;

}

void ScanMatchingI2R::ResetICPAccumulatedPoseAndClearState()
{
  m_x_icp_g = 0; m_y_icp_g = 0; m_yaw_icp_g = 0;
  m_first_time = true;
  m_m1.clear();
  m_m2.clear();
}

void ScanMatchingI2R::ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  VPointCloud::Ptr cloud(new VPointCloud());
  pcl::fromROSMsg(*cloud_msg , *cloud);

  if (m_first_time)
  {
    m_first_time = false;
    transferPclPointCloudToXYPointsMap(cloud, &m_m1);
  }
  else
  {
    transferPclPointCloudToXYPointsMap(cloud, &m_m2);

    CPose2D	initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
    float	runningTime;
    CICP::TReturnInfo	info;
    CPosePDFPtr pdf = m_ICP.Align(
	&m_m1,
	&m_m2,
	initialPose,
	&runningTime,
	(void*)&info);

    printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
	runningTime*1000,
	info.nIterations,
	runningTime*1000.0f/info.nIterations,
	info.goodness*100 );

    cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

    CPosePDFGaussian  gPdf;
    gPdf.copyFrom(*pdf);
    CPosePDFGaussianInf gInf(gPdf);
    mrpt::math::CMatrixDouble33 information_matrix;
    gInf.getInformationMatrix(information_matrix);


    cout << "Covariance of estimation: " << endl << gPdf.cov << endl;
    cout << "Information of estimation: " << endl << information_matrix << endl;

    cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
    cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
    cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;

    mrpt::math::CVectorDouble icp_result;
    pdf->getMeanVal().getAsVector(icp_result);

    geometry_msgs::PoseWithCovarianceStamped pose_tobe_published;
    double x_icp =  cos(-icp_result[2])*icp_result[0] + sin(-icp_result[2])*icp_result[1] ;
    double y_icp = -sin(-icp_result[2])*icp_result[0] + cos(-icp_result[2])*icp_result[1] ;
    m_x_icp_g =  cos(-m_yaw_icp_g)*x_icp + sin(-m_yaw_icp_g)*y_icp + m_x_icp_g;
    m_y_icp_g = -sin(-m_yaw_icp_g)*x_icp + cos(-m_yaw_icp_g)*y_icp + m_y_icp_g;
    m_yaw_icp_g = m_yaw_icp_g + icp_result[2];
    cout << m_x_icp_g << " " << m_y_icp_g << " " << m_yaw_icp_g << endl;
    pose_tobe_published.pose.pose.position.x = m_x_icp_g;
    pose_tobe_published.pose.pose.position.y = m_y_icp_g;
    pose_tobe_published.pose.covariance[0] = information_matrix(0,0);
    pose_tobe_published.pose.covariance[1] = information_matrix(0,1);
    pose_tobe_published.pose.covariance[2] = information_matrix(1,1);
    pose_tobe_published.pose.covariance[3] = information_matrix(2,2);
    pose_tobe_published.pose.covariance[4] = information_matrix(0,2);
    pose_tobe_published.pose.covariance[5] = information_matrix(1,2);
    pose_tobe_published.pose.covariance[6] = icp_result[0];
    pose_tobe_published.pose.covariance[7] = icp_result[1];
    pose_tobe_published.pose.covariance[8] = icp_result[2];
    pose_tobe_published.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_yaw_icp_g);
    m_pub_pose.publish(pose_tobe_published);


    m_m1.clear();
    std::vector<float> xs, ys;
    m_m2.getAllPoints(xs, ys);
    m_m1.setAllPoints(xs, ys);

  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "icp_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(20);

  ScanMatchingI2R scan_matching;

  while (ros::ok()){ros::spinOnce();r.sleep();}
  return 0;
}
