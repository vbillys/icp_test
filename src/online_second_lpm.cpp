#include "icp_test/online_second_lpm.h"


SecondLPM::SecondLPM(string world_frame): m_g_cfg_ifs("default-convert.yaml")  ,   m_p_g_dpf(new PM::DataPointsFilters(m_g_cfg_ifs)), m_g_lpm_Tm_accum(Eigen::Matrix4f::Identity())
{
  m_sub_globalmap = m_nh.subscribe ("icp_globalmap", 1 , &SecondLPM::ProcessGlobalMapCloud, this);
  //m_sub_lpm       = m_nh.subscribe ("lpm_odometry" , 2 , &SecondLPM::ProcessLpmOdometry, this);
  m_sub_localmap  = m_nh.subscribe ("lpm_localmap" , 1 , &SecondLPM::ProcessLocalMapCloud, this);
  m_sub_localmap_last_lpm = m_nh.subscribe ("lpm_last_for_localmap", 1, &SecondLPM::ProcessLastLpmPoseMapped, this);
  m_pub_debug_global_map = m_nh.advertise<sensor_msgs::PointCloud2>( "lpm_debug_globalmap", 1);
  m_pub_debug_local_map  = m_nh.advertise<sensor_msgs::PointCloud2>( "lpm_debug_localmap" , 1);
  m_world_frame = world_frame;
  m_global_map_received = false;
  if (m_g_cfg_ifs.fail()) {std::cout << "can't load icp config file...exiting\n"; m_is_valid = false;}
  else { m_is_valid = true; }
  ifstream ifs("icp-config-global.yaml");
  if (!ifs.good()) {std::cout << "can't load icp config file...exiting\n"; m_is_valid = false;}
  else
  {
    m_icp_3d.loadFromYaml(ifs);
    //m_icp_3d.setDefault();
  }
  m_worst_timing = 0;
}

bool SecondLPM::isValid()
{
  return m_is_valid;
}

void SecondLPM::ProcessLastLpmPoseMapped(const geometry_msgs::PosePtr& pose_msg)
{
  m_last_lpm_mapped_pose_msg = *pose_msg;
}

void SecondLPM::ProcessLpmOdometry(const nav_msgs::OdometryPtr& odom_msg)
{

  // just publish lpm_global correction with same rate as lpm odom
  tf::Transform lpm_global_correction_transform;
  //lpm_global_correction_transform.setOrigin(origin);
  lpm_global_correction_transform.setOrigin(tf::Vector3(0,0,0));
  //lpm_global_correction_transform.setRotation(tfqt);
  lpm_global_correction_transform.setRotation(tf::Quaternion::getIdentity());
  m_t_br.sendTransform(tf::StampedTransform(lpm_global_correction_transform, ros::Time::now()+ros::Duration(3*m_c_worst_time_between_global_loams) , m_world_frame, "lpm_global_correction"));

}

void SecondLPM::publishNecessaries()
{
  // just publish lpm_global correction with same rate as lpm odom
  tf::Transform lpm_global_correction_transform;
  //lpm_global_correction_transform.setOrigin(origin);
  lpm_global_correction_transform.setOrigin(tf::Vector3(0,0,0));
  //lpm_global_correction_transform.setRotation(tfqt);
  lpm_global_correction_transform.setRotation(tf::Quaternion::getIdentity());
  m_t_br.sendTransform(tf::StampedTransform(lpm_global_correction_transform, ros::Time::now()+ros::Duration(3*m_c_worst_time_between_global_loams) , m_world_frame, "lpm_global_correction"));
}

void SecondLPM::ProcessGlobalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  m_global_map_msg = cloud_msg;
  pcl::fromROSMsg(*cloud_msg, m_global_map_cloud);
  std::cout << "Global Map Received!\n";
  m_global_map_filtered = VPointCloud::Ptr(new VPointCloud());
  FilterPointCloud(m_global_map_cloud, m_global_map_filtered);
  
  std::cout << "Done filtered.\n";
  m_global_map_received = true;
}

void SecondLPM::FilterPointCloud(VPointCloud & input_cloud, VPointCloud::Ptr  output_cloud)
{
  sensor_msgs::PointCloud2 input_cloud_pcl;//(new PointCloud());
  pcl::toROSMsg(input_cloud, input_cloud_pcl);
  m_dp_global = DP (PointMatcher_ros::rosMsgToPointMatcherCloud<float>(input_cloud_pcl));
  m_p_g_dpf->apply(m_dp_global);
  sensor_msgs::PointCloud2 cloud_filtered_ros = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(m_dp_global, "world", ros::Time::now());
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pcl (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_filtered_ros , *output_cloud); 
  //transferPclPointCloudXYZToXYPointsMap(cloud_filtered_pcl, &m_filtered_map);
}

void SecondLPM::GetEigenLastLpmPose(Eigen::Affine3f & affine)
{
  tf::Transform tf_last_mapped_lpm_pose;
  tf::poseMsgToTF(m_last_lpm_mapped_pose_msg, tf_last_mapped_lpm_pose);
  Eigen::Affine3d d_eigen_last_mapped_lpm_pose;
  tf::transformTFToEigen(tf_last_mapped_lpm_pose, d_eigen_last_mapped_lpm_pose);
  affine = Eigen::Affine3f (d_eigen_last_mapped_lpm_pose.cast<float>());
}

void SecondLPM::TransformCloud(Eigen::Affine3f affine, VPointCloud & cloud, VPointCloud::Ptr& cloud_transformed)
{
  //cloud_transformed = VPointCloud::Ptr(new VPointCloud());
  for (VPointCloud::iterator it = cloud.begin(); it < cloud.end(); it++)
  {
    Eigen::Vector3f point_eigen(it->x, it->y, it->z);
    point_eigen = affine*point_eigen;
    VPoint point = {point_eigen(0), point_eigen(1), point_eigen(2)};
    cloud_transformed->push_back(point);
  }
  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
  m_dp_global_transformed = rigidTrans->compute(m_dp_global, affine.matrix());
}

//void SecondLPM::DoGlobalICP(sensor_msgs::PointCloud2 & iref, const sensor_msgs::PointCloud2ConstPtr idata)
void SecondLPM::DoGlobalICP(sensor_msgs::PointCloud2 & iref, sensor_msgs::PointCloud2 & idata, Eigen::Matrix4f & eigen_init)
{
  DP data(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(idata));
  DP ref(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(iref));
  //DP ref(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*m_global_map_msg));

  ros::Time tic = ros::Time::now();
  //m_T_lpm = m_icp_3d(data, ref);
  //m_T_lpm = m_icp_3d(data, m_dp_global_transformed);
  PM::TransformationParameters T_init(eigen_init);
  //m_T_lpm = m_icp_3d(data, m_dp_global);
  m_T_lpm = m_icp_3d(data, m_dp_global, T_init);
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

void SecondLPM::ProcessLocalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (false == m_global_map_received)
  {
    ROS_INFO("global map not received yet!!");
    return;
  }
  Eigen::Affine3f f_eigen_last_mapped_lpm_pose;
  GetEigenLastLpmPose(f_eigen_last_mapped_lpm_pose);
  VPointCloud::Ptr global_cloud_transformed(new VPointCloud());
  //TransformCloud(f_eigen_last_mapped_lpm_pose.inverse(),m_global_map_cloud, global_cloud_transformed);
  StampPclMsg<VPointCloud::Ptr>(global_cloud_transformed);
  TransformCloud(f_eigen_last_mapped_lpm_pose.inverse(),*m_global_map_filtered, global_cloud_transformed);
  m_pub_debug_global_map.publish(global_cloud_transformed);
  //

  // using ros to pointmatcher
  sensor_msgs::PointCloud2 ref, data;
  //VPointCloud data_pcl;
  VPointCloud::Ptr data_pcl(new VPointCloud());
  //pcl::fromROSMsg(*cloud_msg, data_pcl);
  pcl::fromROSMsg(*cloud_msg, *data_pcl);

  StampPclMsg<VPointCloud::Ptr>(data_pcl);
  m_pub_debug_local_map.publish(data_pcl);

  //pcl::toROSMsg(data_pcl, data);
  pcl::toROSMsg(*data_pcl, data);
  pcl::toROSMsg(*global_cloud_transformed, ref);
  DoGlobalICP(ref, data, f_eigen_last_mapped_lpm_pose.matrix());
}


template<typename T> void SecondLPM::StampPclMsg(T & pcl_msg)
{
  //pcl_msg->header.stamp = pcl_conversions::toPCL(m_cloud_msg->header).stamp;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  //header.frame_id = m_cloud_msg->header.frame_id;
  //header.frame_id = "lpm";
  header.frame_id = "world";
  pcl_conversions::toPCL(header, pcl_msg->header);
  //pcl_msg->header.frame_id = m_cloud_msg->header.frame_id;
  pcl_msg->height = 1;
}

Eigen::Matrix4f SecondLPM::GetTFAsEigen(string name)
{
  tf::StampedTransform transfrom;
  bool success = GetTF(transfrom, name);
  Eigen::Affine3d eigen_transform;
  if (success)
  {
    tf::transformTFToEigen (transfrom, eigen_transform);
  }
  else
  {
    // if no tf avail, set identity!
    eigen_transform.setIdentity();
  }
  return eigen_transform.matrix().cast<float>();
}

bool SecondLPM::GetTF(tf::StampedTransform & otf, string name)
{
  try{
    //m_listener.lookupTransform("/world", "/odom",  
    //ros::Time(0), otf);
    //m_listener.lookupTransform("/velodyne", "/odom",  
    //ros::Time(0), otf);
    m_listener.lookupTransform(m_world_frame, name,  
	ros::Time(0), otf);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "second_lpm_node");
  ros::NodeHandle nh;//("~");
  ros::Rate r(20);

  SecondLPM second_lpm("world");
  if (!second_lpm.isValid()) exit(1);

  while (ros::ok()){second_lpm.publishNecessaries();ros::spinOnce();r.sleep();}
  return 0;
}
