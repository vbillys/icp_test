#include "icp_test/online_second_icp.h"

SecondICP::SecondICP()
{

  m_sub_localmap = m_nh.subscribe ("icp_localmap", 1 , &SecondICP::ProcessLocalMapCloud, this);
  m_sub_globalmap = m_nh.subscribe ("icp_globalmap", 1 , &SecondICP::ProcessGlobalMapCloud, this);
  m_sub_pose     = m_nh.subscribe ("icp_pose2d", 1 , &SecondICP::ProcessOdomPose, this);
  m_pub_repose   = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "icp_repose2d", 10);
  m_pub_debug_global_icp = m_nh.advertise<sensor_msgs::PointCloud2>( "icp_globalmap_chosen", 10);
  m_pub_debug_local_crop = m_nh.advertise<sensor_msgs::PointCloud2>( "icp_local_map_chosen", 10);
  SetDefaultICPOptions();
  ResetAndClearState();
}

void SecondICP::SetDefaultICPOptions()
{
  m_ICP_method = (int) icpClassic;
  m_ICP.options.ICP_algorithm = (TICPAlgorithm)m_ICP_method;
  m_ICP.options.maxIterations			= m_c_max_iter;
  m_ICP.options.thresholdAng			= DEG2RAD(10.0f);
  m_ICP.options.thresholdDist			= 0.75f;
  m_ICP.options.ALFA					= m_c_icp_alpha; //.7f; //0.8f;
  m_ICP.options.smallestThresholdDist	= 0.05f;
  m_ICP.options.doRANSAC = false;

}

void SecondICP::ResetAndClearState()
{
  ICPTools::zeroPose2D(m_predicted_pose);
  ICPTools::zeroPose2D(m_repose);
  ICPTools::zeroPose2D(m_last_odom);
  ICPTools::zeroPose2D(m_odom);
  ICPTools::zeroPose2D(m_odom_not_cached);

  m_retransform = ICPTools::calcTransformFromPose2D(m_repose);

}

void SecondICP::DoKDTreeSearch()
{
  ros::Time tstart = ros::Time::now();
  //m_global_map.kdTreeNClosestPoint2DIdx(m_last_odom.x ,m_last_odom.y, 10040, m_out_idx, m_out_dist);
  //m_global_map.kdTreeRadiusSearch2D(m_last_odom.x ,m_last_odom.y, m_c_globalmap_crop, m_tree_pair);
  //m_global_map.kdTreeRadiusSearch2D(m_last_odom.x+m_repose.x ,m_last_odom.y+m_repose.y, m_c_globalmap_crop, m_tree_pair);
  m_global_map.kdTreeRadiusSearch2D(m_predicted_pose.x ,m_predicted_pose.y, m_c_globalmap_crop, m_tree_pair);
  //int idx = 0;
  //int size_before = m_out_idx.size();
  int size_before = m_tree_pair.size();
  //for (vector<size_t>::iterator it = m_out_idx.begin() ; it<m_out_idx.end(); it++ , idx++)
  //{
    //if (m_out_dist[idx]>14) m_out_idx.erase(it);
  //}
  ros::Time tend = ros::Time::now();
  float dur = (tend - tstart).toSec();
  //int size_after = m_out_idx.size();
  //cout << endl << dur << endl << size_before << " " << size_after << endl;
  cout << endl << dur << endl << size_before << endl;

}

void SecondICP::UpdateRepose()
{
  // need make sure tf convention is right
  m_repose.yaw = m_icp_result[2]-m_odom.yaw;
  ICPTools::Point2D rot_term = ICPTools::transformPoint2D({m_odom.x, m_odom.y}, {0, 0, m_repose.yaw});
  m_repose.x = m_icp_result[0]-rot_term.x;
  m_repose.y = m_icp_result[1]-rot_term.y;

  //ICPTools::Point2D rot_term = transformPoint2D({-m_odom.x, -m_odom.y}, {0, 0, m_repose.yaw});
  //rot_term = ICPTools::transformPoint2D({rot_term.x, rot_term.y}, {m_odom.x, m_odom.y,0});
  //m_repose.x += rot_term.x;
  //m_repose.y += rot_term.y;
  m_retransform = ICPTools::calcTransformFromPose2D(m_repose);
  m_last_odom = m_odom;
}

void SecondICP::ProcessLocalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    m_odom = m_odom_not_cached;
    ProposeNewVehicleLocation();

    //DoKDTreeSearch();
    //PublishDebugKDTreeSearch();

    m_local_cloud = VPointCloud::Ptr(new VPointCloud());
    pcl::fromROSMsg(*cloud_msg, *m_local_cloud);

    //CropLocalCloud();
    PublishDebugCropLocalCloud();

    //DoICP();
    DoICPWithWholeGlobalMap();
    printOutStatus();
    UpdateRepose();
    // store (mark) that this is the current odometry to be used on next calc. as initial state for ICP level 2
    //m_last_odom = m_odom;

}

void SecondICP::ProposeNewVehicleLocation()
{
  //CPose2D		initialPose(m_last_odom.x,m_last_odom.y,m_last_odom.yaw);
  //
  //ICPTools::Point2D xy_remain = {m_odom.x - m_last_odom.x, m_odom.y - m_last_odom.y};
  //ICPTools::Pose2D pose_remain = {0, 0, m_repose.yaw};
  //ICPTools::Point2D uncorrected_odom = transformPoint2D(xy_remain, pose_remain);
  //uncorrected_odom = transformPoint2D(uncorrected_odom, {m_last_odom.x, m_last_odom.y,0});
  ////uncorrected_odom = transformPoint2D(uncorrected_odom, {0 , 0, m_last_odom.yaw});

  //m_predicted_pose.x   = uncorrected_odom.x+m_repose.x;
  //m_predicted_pose.y   = uncorrected_odom.y+m_repose.y;
  //m_predicted_pose.yaw = m_odom.yaw + m_repose.yaw;

  ICPTools::Pose2D pose_remain (m_odom.x - m_last_odom.x, m_odom.y - m_last_odom.y, m_odom.yaw - m_last_odom.yaw);
  m_predicted_pose.x   = m_last_odom.x   + m_repose.x   + pose_remain.x;
  m_predicted_pose.y   = m_last_odom.y   + m_repose.y   + pose_remain.y;
  m_predicted_pose.yaw = m_last_odom.yaw + m_repose.yaw + pose_remain.yaw;

}

void SecondICP::DoICP()
{
  //CPose2D		initialPose(m_last_odom.x+uncorrected_odom.x+m_repose.x,m_last_odom.y+uncorrected_odom.y+m_repose.y,m_odom.yaw - m_last_odom.yaw+m_repose.yaw);
  //CPose2D		initialPose(uncorrected_odom.x+m_repose.x,uncorrected_odom.y+m_repose.y,m_odom.yaw + m_repose.yaw);
  CPose2D		initialPose(m_predicted_pose.x,m_predicted_pose.y,m_predicted_pose.yaw);
  CSimplePointsMap local_map;
  transferPclPointCloudToXYPointsMap(m_local_cloud, &local_map);
  m_pdf = m_ICP.Align(
      &m_current_global_map,
      &local_map,
      initialPose,
      &m_runningTime,
      (void*)&m_info);
  m_pdf->getMeanVal().getAsVector(m_icp_result);

}

void SecondICP::DoICPWithWholeGlobalMap()
{
  //CPose2D		initialPose(m_last_odom.x+uncorrected_odom.x+m_repose.x,m_last_odom.y+uncorrected_odom.y+m_repose.y,m_odom.yaw - m_last_odom.yaw+m_repose.yaw);
  //CPose2D		initialPose(uncorrected_odom.x+m_repose.x,uncorrected_odom.y+m_repose.y,m_odom.yaw + m_repose.yaw);
  CPose2D		initialPose(m_predicted_pose.x,m_predicted_pose.y,m_predicted_pose.yaw);
  CSimplePointsMap local_map;
  transferPclPointCloudToXYPointsMap(m_local_cloud, &local_map);
  m_pdf = m_ICP.Align(
      &m_global_map,
      &local_map,
      initialPose,
      &m_runningTime,
      (void*)&m_info);
  m_pdf->getMeanVal().getAsVector(m_icp_result);

}

void SecondICP::printOutStatus()
{
  printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
      m_runningTime*1000,
      m_info.nIterations,
      m_runningTime*1000.0f/m_info.nIterations,
      m_info.goodness*100 );

  cout << "Mean of estimation: " << m_pdf->getMeanVal() << endl<< endl;

  //cout << "Covariance of estimation: " << endl << m_gPdf.cov << endl;
  //cout << "Information of estimation: " << endl << m_information_matrix << endl;

  //cout << " std(x): " << sqrt( m_gPdf.cov(0,0) ) << endl;
  //cout << " std(y): " << sqrt( m_gPdf.cov(1,1) ) << endl;
  //cout << " std(phi): " << RAD2DEG(sqrt( m_gPdf.cov(2,2) )) << " (deg)" << endl;
  //cout << m_x_icp_g << " " << m_y_icp_g << " " << m_yaw_icp_g << endl;
}

void SecondICP::CropLocalCloud()
{
  VPointCloud::Ptr cropped_cloud(new VPointCloud());
  for (VPointCloud::iterator it = m_local_cloud->begin(); it < m_local_cloud->end(); it++)
  {
    ICPTools::Point2D point (it->x , it->y);
    if (ICPTools::calcVectorLength(point) < m_c_localmap_crop)
    {
      //m_local_cloud->erase(it);
      VPoint point_to_add;
      point_to_add.x = it->x; point_to_add.y = it->y;
      cropped_cloud->push_back(point_to_add);
    }
  }
  m_local_cloud = cropped_cloud;
}

void SecondICP::PublishDebugCropLocalCloud()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "ibeo";
  pcl_conversions::toPCL(header, m_local_cloud->header);
  m_local_cloud->height = 1;
  m_pub_debug_local_crop.publish(m_local_cloud);
}

//void SecondICP::UseAllGlobalMap()
//{

//}

void SecondICP::PublishDebugKDTreeSearch()
{
  VPointCloud::Ptr debug_cloud(new VPointCloud());
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "map";
  pcl_conversions::toPCL(header, debug_cloud->header);
  debug_cloud->height = 1;

  //CSimplePointsMap m;
  std::vector<float> xs, ys; 
  //for (vector<size_t>::iterator it = m_out_idx.begin() ; it < m_out_idx.end(); it++ )
  for (vector<std::pair< size_t, float > >::iterator it = m_tree_pair.begin() ; it < m_tree_pair.end(); it++ )
  {
    float _x, _y;
    //m_global_map.getPoint(*it, _x, _y);
    m_global_map.getPoint(it->first, _x, _y);
    xs.push_back(_x); ys.push_back(_y);
  }
  m_current_global_map.setAllPoints(xs, ys);
  transferPointsMapToPointCloud(&m_current_global_map, debug_cloud);
  m_pub_debug_global_icp.publish(debug_cloud);
}

void SecondICP::ProcessGlobalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //store only, for local map ICP
  pcl::PointCloud<pcl::PointXYZ>::Ptr globalmap_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *globalmap_cloud);
  transferPclPointCloudXYZToXYPointsMap(globalmap_cloud, &m_global_map);
  cout << "GOT MAP! \nProcessing KDTree for this map\n";
  vector<size_t> out_idx; vector<float> out_dist;
  ros::Time tstart = ros::Time::now();
  m_global_map.kdTreeNClosestPoint2DIdx(0 ,0, 40, out_idx, out_dist);
  ros::Time tend = ros::Time::now();
  cout << "KDTrees-ed!!\n";
  int idx =0;
  for (vector<size_t>::iterator it = out_idx.begin() ; it<out_idx.end(); it++ , idx++)
  {
    cout << *it << " " << out_dist[idx] << " ";
  }
  cout.flush();
  float dur = (tend - tstart).toSec();
  cout << endl << dur << endl;
  //tstart = ros::Time::now();
  //m_global_map.kdTreeNClosestPoint2DIdx(0 ,0, 10040, out_idx, out_dist);
  //idx = 0;
  //int size_before = out_idx.size();
  //for (vector<size_t>::iterator it = out_idx.begin() ; it<out_idx.end(); it++ , idx++)
  //{
    //if (out_dist[idx]>20) out_idx.erase(it);
  //}
  //tend = ros::Time::now();
  //dur = (tend - tstart).toSec();
  //int size_after = out_idx.size();
  //cout << endl << dur << endl << size_before << " " << size_after << endl;
  DoKDTreeSearch();
}


void SecondICP::ProcessOdomPose(const geometry_msgs::PoseWithCovarianceStampedPtr& pose_msg)
{
  m_odom_not_cached.x = pose_msg->pose.pose.position.x;
  m_odom_not_cached.y = pose_msg->pose.pose.position.y;
  tf::Quaternion quat(pose_msg->pose.pose.orientation.x,pose_msg->pose.pose.orientation.y,pose_msg->pose.pose.orientation.z,pose_msg->pose.pose.orientation.w);    
  m_odom_not_cached.yaw = tf::getYaw(quat);
  // just keep tranform update
  
  m_t_br.sendTransform(tf::StampedTransform(m_retransform, ros::Time::now(), "map", "odom"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "second_icp_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(20);

  SecondICP second_icp;

  while (ros::ok()){ros::spinOnce();r.sleep();}
  return 0;
}
