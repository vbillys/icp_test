#include "icp_test/online_lpm_node.h"

ScanMatching3D::ScanMatching3D(string world_frame): m_g_cfg_ifs("default-convert.yaml")  ,   m_p_g_dpf(new PM::DataPointsFilters(m_g_cfg_ifs)), m_g_lpm_Tm_accum(Eigen::Matrix4f::Identity())
{

  m_ICP_method = (int) icpClassic;
  m_sub = m_nh.subscribe ("velodyne_points", 1 , &ScanMatching3D::ProcessPointCloud3D, this);
  //m_pub_pose = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "lpm_pose3d", 10);
  m_pub_lpm_odom = m_nh.advertise<nav_msgs::Odometry>( "lpm_odometry", 10);
  m_pub_localmap = m_nh.advertise<sensor_msgs::PointCloud2>( "lpm_localmap", 2);
  SetDefaultICPOptions();
  ResetICPAccumulatedPoseAndClearState();
  //m_g_cfg_ifs = std::ifstream("default-convert.yaml");
  //m_g_dpf = PM::DataPointsFilters(m_g_cfg_ifs);
  //m_g_cfg_ifs.open("default-convert.yaml") ; 
  //m_p_g_dpf = boost::scoped_ptr<PM::DataPointsFilters>(new PM::DataPointsFilters(m_g_cfg_ifs));
  if (m_g_cfg_ifs.fail()) {std::cout << "can't load icp config file...exiting\n"; m_is_valid = false;}
  else { m_is_valid = true; }
  ifstream ifs("icp-config.yaml");
  if (!ifs.good()) {std::cout << "can't load icp config file...exiting\n"; m_is_valid = false;}
  else
  {
    m_icp_3d.loadFromYaml(ifs);
    //m_icp_3d.setDefault();
  }
  m_worst_timing = 0;
  m_world_frame = world_frame;
}

bool ScanMatching3D::isValid()
{
  return m_is_valid;
}

void ScanMatching3D::SetDefaultICPOptions()
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

void ScanMatching3D::ResetICPAccumulatedPoseAndClearState()
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

void ScanMatching3D::DoICP3D(ICPTools::Pose2D & init)
{
  ros::Time tic = ros::Time::now();

  DP data(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*m_cloud_msg));
  DP ref(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*m_m1_3d));
  PM::TransformationParameters T_init(4,4);
  T_init << cos(init.yaw), -sin(init.yaw),0,init.x
    ,sin(init.yaw),  cos(init.yaw),0,init.y
    ,0                      , 0                       ,1,0
    ,0                      , 0                       ,0,1;
  //PM::TransformationParameters T = icp(data, ref);
  m_T_lpm = m_icp_3d(data, ref, T_init);

  ros::Time toc = ros::Time::now();
  float timing = (toc -tic).toSec() * 1000;
  if (timing > m_worst_timing) m_worst_timing = timing;
  using namespace std;
  cout << "Final transformation:" << endl << m_T_lpm << endl;
  cout << timing << " ms. worst: "  << m_worst_timing << " ms" << endl;
  //Eigen::Matrix4f eT(m_T_lpm);
  //m_g_lpm_Tm_accum =  m_g_lpm_Tm_accum * eT;
  m_g_lpm_Tm_accum =  m_g_lpm_Tm_accum * m_T_lpm;
}


void ScanMatching3D::DoICP()
{
  m_pdf = m_ICP.Align(
      &m_m1,
      &m_m2,
      m_initialPose,
      &m_runningTime,
      (void*)&m_info);
  m_pdf->getMeanVal().getAsVector(m_icp_result);
}

void ScanMatching3D::ComputeInformationMatrix()
{
  m_gPdf.copyFrom(*m_pdf);
  CPosePDFGaussianInf gInf(m_gPdf);
  gInf.getInformationMatrix(m_information_matrix);
}

void ScanMatching3D::ComputeAccumulatedPose()
{
  double x_icp =  cos(-m_icp_result[2])*m_icp_result[0] + sin(-m_icp_result[2])*m_icp_result[1] ;
  double y_icp = -sin(-m_icp_result[2])*m_icp_result[0] + cos(-m_icp_result[2])*m_icp_result[1] ;
  m_x_icp_g =  cos(-m_yaw_icp_g)*x_icp + sin(-m_yaw_icp_g)*y_icp + m_x_icp_g;
  m_y_icp_g = -sin(-m_yaw_icp_g)*x_icp + cos(-m_yaw_icp_g)*y_icp + m_y_icp_g;
  m_yaw_icp_g = m_yaw_icp_g + m_icp_result[2];
  m_pose.x = m_x_icp_g; m_pose.y = m_y_icp_g; m_pose.yaw = m_yaw_icp_g;
}

void ScanMatching3D::printOutStatus()
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

void ScanMatching3D::publishPose3D()
{
  tf::Vector3 origin;
  origin.setValue(m_g_lpm_Tm_accum (0,3), m_g_lpm_Tm_accum (1,3), m_g_lpm_Tm_accum (2,3));

  tf::Matrix3x3 tf3d;
  tf3d.setValue(m_g_lpm_Tm_accum (0,0), m_g_lpm_Tm_accum (0,1), m_g_lpm_Tm_accum (0,2), m_g_lpm_Tm_accum (1,0), m_g_lpm_Tm_accum (1,1), m_g_lpm_Tm_accum (1,2), m_g_lpm_Tm_accum (2,0), m_g_lpm_Tm_accum (2,1), m_g_lpm_Tm_accum (2,2));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  //tf::Transform transform;
  m_transform.setOrigin(origin);
  m_transform.setRotation(tfqt);

  static tf::TransformBroadcaster br;
  //br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), "velodyne", "lpm"));
  br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), m_world_frame, "lpm"));


  // translation part only from lpm
  tf::Vector3 atra;
  Eigen::Affine3f eigen_lpm_accum_transform(m_g_lpm_Tm_accum);
  tf::vectorEigenToTF(eigen_lpm_accum_transform.translation().cast<double>(), atra);

  // rotation part only from lpm
  Eigen::Matrix3d odom_accum_rot_eigen;
  tf::matrixTFToEigen(m_g_s_transform.getBasis(), odom_accum_rot_eigen);
  Eigen::Quaternionf quat_eig( Eigen::Affine3f(eigen_lpm_accum_transform).rotation()*odom_accum_rot_eigen.inverse().cast<float>()  );
  tf::quaternionEigenToTF(quat_eig.cast<double>(), tfqt);


  // rotation part side effects from accumulation
  tf::Vector3 stra;
  Eigen::Vector3d aT;
  tf::vectorTFToEigen(m_g_s_transform.getOrigin(), aT);
  tf::vectorEigenToTF(Eigen::Affine3f(eigen_lpm_accum_transform).rotation().cast<double>() * odom_accum_rot_eigen.inverse() * aT,stra);
  origin   = atra - stra;

  //tf::Transform lpm_correct_transform;
  m_lpm_correct_transform.setOrigin(origin);
  //m_lpm_correct_transform.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion no_rot_quat(tf::Quaternion::getIdentity());
  m_lpm_correct_transform.setRotation(tfqt);
  //m_lpm_correct_transform.setRotation(no_rot_quat);
  //br.sendTransform(tf::StampedTransform(m_lpm_correct_transform, ros::Time::now(), "velodyne", "lpm_correction"));
  br.sendTransform(tf::StampedTransform(m_lpm_correct_transform, ros::Time::now()+ros::Duration(m_c_worst_time_between_loams) , m_world_frame, "lpm_correction"));

  nav_msgs::Odometry lpm_odom;
  lpm_odom.header.frame_id= m_world_frame; //"velodyne";
  lpm_odom.child_frame_id = "lpm";
  lpm_odom.header.stamp = ros::Time::now();
  geometry_msgs::Pose lpm_odom_pose;
  tf::poseTFToMsg(m_transform, lpm_odom_pose);
  lpm_odom.pose.pose = lpm_odom_pose;
  m_pub_lpm_odom.publish(lpm_odom);
}


void ScanMatching3D::publishPose()
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

  m_transform.setOrigin(tf::Vector3(m_x_icp_g, m_y_icp_g, 0) );
  tf::Quaternion quat;
  quat.setRPY(0 ,0 ,m_yaw_icp_g);
  m_transform.setRotation(quat);
  m_t_br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), "odom", "ibeo"));

}

void ScanMatching3D::TransferScanDataToRefData()
{
  //m_m1.clear();
  //std::vector<float> xs, ys;
  //m_m2.getAllPoints(xs, ys);
  //m_m1.setAllPoints(xs, ys);
  m_m1 = m_m2;
}

void ScanMatching3D::ComputeStraightDistanceTravelledFromMarkedPose()
{
  m_travelled_dist =  sqrt((m_marked_pose.x - m_pose.x)*(m_marked_pose.x - m_pose.x)
                          +(m_marked_pose.y - m_pose.y)*(m_marked_pose.y - m_pose.y)) ;
}

void ScanMatching3D::MarkPose()
{
  m_marked_pose = m_pose;
}

bool ScanMatching3D::PushToLocalMapUsingGlobalPose(const CSimplePointsMap & input_map) 
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



void ScanMatching3D::GetAllPointsFromLocalMap(vector<ICPTools::Point2D> & points)
{
  for (vector<LocalMap>::iterator it = m_local_maps.begin() ; it < m_local_maps.end(); it++)
  {
    
    std::vector<float> xs, ys;
    (*it).points_set.getAllPoints(xs, ys);
    LocalMap * last_map = &(m_local_maps.back());
    ICPTools::Pose2D relative_pose(it->global_pose.x - last_map->global_pose.x
			   ,it->global_pose.y - last_map->global_pose.y
			   ,it->global_pose.yaw - last_map->global_pose.yaw);
    int i = 0;
    for (vector<float>::iterator iit = xs.begin() ; iit < xs.end() ; iit++, i++)
    {
      ICPTools::Point2D _point2d ( *iit, ys[i] );
      _point2d = ICPTools::transformPoint2D(_point2d, it->global_pose);
      _point2d = ICPTools::transformPoint2D(_point2d, {-last_map->global_pose.x, -last_map->global_pose.y, 0} );
      _point2d = ICPTools::transformPoint2D(_point2d, {0, 0, -last_map->global_pose.yaw} );
      points.push_back(_point2d);
    }
  }
}

void ScanMatching3D::ToPclFromLocalMapFromPoints2D(vector<ICPTools::Point2D> & points, VPointCloud::Ptr & output_cloud)
{
  for (vector<ICPTools::Point2D>::iterator it = points.begin(); it < points.end() ; it++)
  {
    velodyne_pointcloud::PointXYZIR _point; 
    _point.x = (*it).x; _point.y =  (*it).y; _point.z = 0; _point.intensity = 0; _point.ring = 0;
    output_cloud->push_back(_point);
  }
}

template<typename T> void ScanMatching3D::StampPclMsgWithOrigRosMsg(T & pcl_msg)
{
  //pcl_msg->header.stamp = pcl_conversions::toPCL(m_cloud_msg->header).stamp;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = m_cloud_msg->header.frame_id;
  pcl_conversions::toPCL(header, pcl_msg->header);
  //pcl_msg->header.frame_id = m_cloud_msg->header.frame_id;
  pcl_msg->height = 1;
}

void ScanMatching3D::publishLocalMap()
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

void ScanMatching3D::FilterPointCloud(VPointCloud::Ptr & input_cloud)
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

void ScanMatching3D::StorePointerToRosMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  m_cloud_msg = cloud_msg;
}

void ScanMatching3D::GetTF(tf::StampedTransform & otf)
{
  try{
    //m_listener.lookupTransform("/world", "/odom",  
	//ros::Time(0), otf);
    //m_listener.lookupTransform("/velodyne", "/odom",  
	//ros::Time(0), otf);
    m_listener.lookupTransform(m_world_frame, "/odom",  
	ros::Time(0), otf);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void ScanMatching3D::ProcessPointCloud3D(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  VPointCloud::Ptr cloud(new VPointCloud());
  pcl::fromROSMsg(*cloud_msg , *cloud);
  StorePointerToRosMsg(cloud_msg);

  if (m_first_time)
  {
    GetTF(m_g_s_transform);
    m_first_time = false;
    m_m1_3d = cloud_msg;
    m_last_loam_start = ros::Time::now();
  }
  else
  {


    tf::StampedTransform s_transform;
    GetTF(s_transform);
    ICPTools::Pose2D odom_diff = GetOdomDiff(m_g_s_transform, s_transform);

    if (ICPTools::calcDistPose2D(odom_diff) > m_c_distance_loam_update && (ros::Time::now() - m_last_loam_start).toSec() > m_c_worst_time_between_loams )
    {
      DoICP3D(odom_diff);
      m_g_s_transform = s_transform;
      m_m1_3d = cloud_msg;
      m_last_loam_start = ros::Time::now();
    }
    publishPose3D();
  }

}

ICPTools::Pose2D ScanMatching3D::GetOdomDiff(tf::StampedTransform &before, tf::StampedTransform &after)
{
  // look up last odom for initial value
  double last_odom_diff_roll, last_odom_diff_pitch, last_odom_diff_yaw;
  double last_odom_diff_x, last_odom_diff_y, last_odom_diff_z;
  //cout << after;

  last_odom_diff_x = after.getOrigin().x() - before.getOrigin().x();
  last_odom_diff_y = after.getOrigin().y() - before.getOrigin().y();

  tf::Quaternion after_quat = after.getRotation();
  tf::Quaternion before_quat = before.getRotation();
  double after_yaw, before_yaw;
  double roll, pitch, yaw;
  tf::Matrix3x3(after_quat).getRPY(roll, pitch, after_yaw);
  tf::Matrix3x3(before_quat).getRPY(roll, pitch, before_yaw);
  last_odom_diff_yaw = after_yaw - before_yaw;
  cout << "odom_diff: " << last_odom_diff_x << " " << last_odom_diff_y << " " << last_odom_diff_yaw << endl;
  return ICPTools::Pose2D (last_odom_diff_x, last_odom_diff_y, last_odom_diff_yaw);
}


void ScanMatching3D::ProcessPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
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
  ros::init(argc, argv, "lpm_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(20);

  //ScanMatching3D scan_matching("velodyne");
  ScanMatching3D scan_matching("world");
  if (!scan_matching.isValid()) exit(1);

  while (ros::ok()){ros::spinOnce();r.sleep();}
  return 0;
}
