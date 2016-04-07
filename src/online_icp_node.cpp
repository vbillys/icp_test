#include "icp_test/online_icp_node.h"

ScanMatchingI2R::ScanMatchingI2R()
  : m_g_cfg_ifs("default-convert.yaml") , m_g_dpf(m_g_cfg_ifs)
{

  m_ICP_method = (int) icpClassic;
  m_sub = m_nh.subscribe ("ibeo_points", 1 , &ScanMatchingI2R::ProcessPointCloud, this);
  m_pub_pose = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "icp_pose2d", 10);
  m_pub_localmap = m_nh.advertise<sensor_msgs::PointCloud2>( "icp_localmap", 2);
  SetDefaultICPOptions();
  ResetICPAccumulatedPoseAndClearState();
  //m_g_cfg_ifs = std::ifstream("default-convert.yaml");
  //m_g_dpf = PM::DataPointsFilters(m_g_cfg_ifs);
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
  m_initialPose = CPose2D(0.0f,0.0f,(float)DEG2RAD(0.0f));

}

void ScanMatchingI2R::ResetICPAccumulatedPoseAndClearState()
{
  m_x_icp_g = 0; m_y_icp_g = 0; m_yaw_icp_g = 0;
  m_pose.x = 0; m_pose.y = 0; m_pose.yaw = 0;
  m_marked_pose.x = 0; m_marked_pose.y = 0; m_marked_pose.yaw = 0;
  m_travelled_dist = 0;
  m_next_capture_dist = - m_c_distance_between_local_maps;
  m_first_time = true;
  m_m1.clear();
  m_m2.clear();
}

void ScanMatchingI2R::DoICP()
{
  m_pdf = m_ICP.Align(
      &m_m1,
      &m_m2,
      m_initialPose,
      &m_runningTime,
      (void*)&m_info);
  m_pdf->getMeanVal().getAsVector(m_icp_result);
}

void ScanMatchingI2R::ComputeInformationMatrix()
{
  m_gPdf.copyFrom(*m_pdf);
  CPosePDFGaussianInf gInf(m_gPdf);
  gInf.getInformationMatrix(m_information_matrix);
}

void ScanMatchingI2R::ComputeAccumulatedPose()
{
  double x_icp =  cos(-m_icp_result[2])*m_icp_result[0] + sin(-m_icp_result[2])*m_icp_result[1] ;
  double y_icp = -sin(-m_icp_result[2])*m_icp_result[0] + cos(-m_icp_result[2])*m_icp_result[1] ;
  m_x_icp_g =  cos(-m_yaw_icp_g)*x_icp + sin(-m_yaw_icp_g)*y_icp + m_x_icp_g;
  m_y_icp_g = -sin(-m_yaw_icp_g)*x_icp + cos(-m_yaw_icp_g)*y_icp + m_y_icp_g;
  m_yaw_icp_g = m_yaw_icp_g + m_icp_result[2];
  m_pose.x = m_x_icp_g; m_pose.y = m_y_icp_g; m_pose.yaw = m_yaw_icp_g;
}

void ScanMatchingI2R::printOutStatus()
{
  printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
      m_runningTime*1000,
      m_info.nIterations,
      m_runningTime*1000.0f/m_info.nIterations,
      m_info.goodness*100 );

  cout << "Mean of estimation: " << m_pdf->getMeanVal() << endl<< endl;

  cout << "Covariance of estimation: " << endl << m_gPdf.cov << endl;
  cout << "Information of estimation: " << endl << m_information_matrix << endl;

  cout << " std(x): " << sqrt( m_gPdf.cov(0,0) ) << endl;
  cout << " std(y): " << sqrt( m_gPdf.cov(1,1) ) << endl;
  cout << " std(phi): " << RAD2DEG(sqrt( m_gPdf.cov(2,2) )) << " (deg)" << endl;
  cout << m_x_icp_g << " " << m_y_icp_g << " " << m_yaw_icp_g << endl;
}

void ScanMatchingI2R::publishPose()
{

  geometry_msgs::PoseWithCovarianceStamped pose_tobe_published;
  pose_tobe_published.pose.pose.position.x = m_x_icp_g;
  pose_tobe_published.pose.pose.position.y = m_y_icp_g;
  pose_tobe_published.pose.covariance[0] = m_information_matrix(0,0);
  pose_tobe_published.pose.covariance[1] = m_information_matrix(0,1);
  pose_tobe_published.pose.covariance[2] = m_information_matrix(1,1);
  pose_tobe_published.pose.covariance[3] = m_information_matrix(2,2);
  pose_tobe_published.pose.covariance[4] = m_information_matrix(0,2);
  pose_tobe_published.pose.covariance[5] = m_information_matrix(1,2);
  pose_tobe_published.pose.covariance[6] = m_icp_result[0];
  pose_tobe_published.pose.covariance[7] = m_icp_result[1];
  pose_tobe_published.pose.covariance[8] = m_icp_result[2];
  pose_tobe_published.pose.pose.orientation = tf::createQuaternionMsgFromYaw(m_yaw_icp_g);
  m_pub_pose.publish(pose_tobe_published);
}

void ScanMatchingI2R::TransferScanDataToRefData()
{
  //m_m1.clear();
  //std::vector<float> xs, ys;
  //m_m2.getAllPoints(xs, ys);
  //m_m1.setAllPoints(xs, ys);
  m_m1 = m_m2;
}

void ScanMatchingI2R::ComputeStraightDistanceTravelledFromMarkedPose()
{
  m_travelled_dist =  sqrt((m_marked_pose.x - m_pose.x)*(m_marked_pose.x - m_pose.x)
                          +(m_marked_pose.y - m_pose.y)*(m_marked_pose.y - m_pose.y)) ;
}

void ScanMatchingI2R::MarkPose()
{
  m_marked_pose = m_pose;
}

bool ScanMatchingI2R::PushToLocalMapUsingGlobalPose(const CSimplePointsMap & input_map) 
{ 
  //cout << "PushLocalMap called\n";
  bool updated_maps = false;
  ComputeStraightDistanceTravelledFromMarkedPose();
  //cout << m_travelled_dist << " " << m_local_maps.size() << endl;
  //if (m_travelled_dist >= m_next_capture_dist)
  if (m_travelled_dist >= m_c_distance_between_local_maps || m_first_time)
  {
    //while (m_next_capture_dist <= m_travelled_dist) m_next_capture_dist += m_c_distance_between_local_maps;
    MarkPose();
    LocalMap map;
    map.points_set = input_map;
    map.global_pose = m_pose;
    m_local_maps.push_back(map);
    //cout << "map added\n";
    if (m_local_maps.size() > m_c_max_no_of_local_maps)
    {
      updated_maps = true;
      //remove oldest points_set
      m_local_maps.erase (m_local_maps.begin());
      //cout << "local maps full, shifting window\n";

    }
  }
  return updated_maps;
}



void ScanMatchingI2R::GetAllPointsFromLocalMap(vector<ICPTools::Point2D> & points)
{
  for (vector<LocalMap>::iterator it = m_local_maps.begin() ; it < m_local_maps.end(); it++)
  {
    
    std::vector<float> xs, ys;
    (*it).points_set.getAllPoints(xs, ys);
    LocalMap * last_map = &(m_local_maps.back());
    ICPTools::Pose2D relative_pose = {it->global_pose.x - last_map->global_pose.x
			   ,it->global_pose.y - last_map->global_pose.y
			   ,it->global_pose.yaw - last_map->global_pose.yaw};
    int i = 0;
    for (vector<float>::iterator iit = xs.begin() ; iit < xs.end() ; iit++, i++)
    {
      ICPTools::Point2D _point2d = { *iit, ys[i] };
      _point2d = ICPTools::transformPoint2D(_point2d, it->global_pose);
      _point2d = ICPTools::transformPoint2D(_point2d, {-last_map->global_pose.x, -last_map->global_pose.y, 0} );
      _point2d = ICPTools::transformPoint2D(_point2d, {0, 0, -last_map->global_pose.yaw} );
      points.push_back(_point2d);
    }
  }
}

void ScanMatchingI2R::ToPclFromLocalMapFromPoints2D(vector<ICPTools::Point2D> & points, VPointCloud::Ptr & output_cloud)
{
  for (vector<ICPTools::Point2D>::iterator it = points.begin(); it < points.end() ; it++)
  {
    velodyne_pointcloud::PointXYZIR _point; 
    _point.x = (*it).x; _point.y =  (*it).y; _point.z = 0; _point.intensity = 0; _point.ring = 0;
    output_cloud->push_back(_point);
  }
}

template<typename T> void ScanMatchingI2R::StampPclMsgWithOrigRosMsg(T & pcl_msg)
{
  //pcl_msg->header.stamp = pcl_conversions::toPCL(m_cloud_msg->header).stamp;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = m_cloud_msg->header.frame_id;
  pcl_conversions::toPCL(header, pcl_msg->header);
  //pcl_msg->header.frame_id = m_cloud_msg->header.frame_id;
  pcl_msg->height = 1;
}

void ScanMatchingI2R::publishLocalMap()
{
  vector<ICPTools::Point2D> all_points;
  GetAllPointsFromLocalMap(all_points);
  VPointCloud::Ptr all_points_pcl(new VPointCloud());
  StampPclMsgWithOrigRosMsg<VPointCloud::Ptr>(all_points_pcl);
  ToPclFromLocalMapFromPoints2D(all_points, all_points_pcl);
  m_pub_localmap.publish(all_points_pcl);
  //cout << " KENTUTTTTTTTTT: "<< all_points.size() << " " << all_points_pcl->points.size() << endl;
  //cout << all_points_pcl->points[0].x << " " << all_points_pcl->points[0].y << endl;
}

void ScanMatchingI2R::FilterPointCloud(VPointCloud::Ptr & input_cloud)
{
  sensor_msgs::PointCloud2 point_cloud_total_pcl;//(new PointCloud());
  pcl::toROSMsg(*input_cloud, point_cloud_total_pcl);
  DP mapPointCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(point_cloud_total_pcl));
  m_g_dpf.apply(mapPointCloud);
  sensor_msgs::PointCloud2 cloud_filtered_ros = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, "ibeo", ros::Time::now());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pcl (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_filtered_ros , *cloud_filtered_pcl); 
  transferPclPointCloudXYZToXYPointsMap(cloud_filtered_pcl, &m_filtered_map);
}

void ScanMatchingI2R::StorePointerToRosMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  m_cloud_msg = cloud_msg;
}

void ScanMatchingI2R::ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  VPointCloud::Ptr cloud(new VPointCloud());
  pcl::fromROSMsg(*cloud_msg , *cloud);
  StorePointerToRosMsg(cloud_msg);

  if (m_first_time)
  {
    m_first_time = false;
    transferPclPointCloudToXYPointsMap(cloud, &m_m1);
    FilterPointCloud(cloud);
    //PushToLocalMapUsingGlobalPose(m_m1);
    PushToLocalMapUsingGlobalPose(m_filtered_map);
  }
  else
  {
    transferPclPointCloudToXYPointsMap(cloud, &m_m2);

    DoICP();
    TransferScanDataToRefData();

    ComputeAccumulatedPose();
    ComputeInformationMatrix();

    FilterPointCloud(cloud);
    //bool local_map_updated = PushToLocalMapUsingGlobalPose(m_m2);
    bool local_map_updated = PushToLocalMapUsingGlobalPose(m_filtered_map);

    printOutStatus();

    publishPose();
    if (local_map_updated) publishLocalMap();
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
