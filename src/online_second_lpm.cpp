#include "icp_test/online_second_lpm.h"


SecondLPM::SecondLPM(string world_frame)
{
  m_sub_globalmap = m_nh.subscribe ("icp_globalmap", 1 , &SecondLPM::ProcessGlobalMapCloud, this);
  m_sub_lpm       = m_nh.subscribe ("lpm_odometry" , 2 , &SecondLPM::ProcessLpmOdometry, this);
  m_sub_localmap  = m_nh.subscribe ("lpm_localmap" , 1 , &SecondLPM::ProcessLocalMapCloud, this);
  m_world_frame = world_frame;
  m_global_map_received = false;
}


void SecondLPM::ProcessLpmOdometry(const nav_msgs::OdometryPtr& odom_msg)
{
  // just publish lpm_global correction with same rate as lpm odom
  tf::Transform lpm_global_transform;
  //lpm_global_transform.setOrigin(origin);
  lpm_global_transform.setOrigin(tf::Vector3(0,0,0));
  //tf::Quaternion no_rot_quat(tf::Quaternion::getIdentity());
  //m_lpm_correct_transform.setRotation(tfqt);
  lpm_global_transform.setRotation(tf::Quaternion::getIdentity());
  //br.sendTransform(tf::StampedTransform(m_lpm_correct_transform, ros::Time::now(), "velodyne", "lpm_correction"));
  m_t_br.sendTransform(tf::StampedTransform(lpm_global_transform, ros::Time::now()+ros::Duration(3*m_c_worst_time_between_global_loams) , m_world_frame, "lpm_global_correction"));
}


void SecondLPM::ProcessGlobalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  m_global_map_msg = cloud_msg;
  std::cout << "Global Map Received!\n";
  m_global_map_received = true;
}


void SecondLPM::ProcessLocalMapCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (false == m_global_map_received)
  {
    ROS_INFO("global map not received yet!!");
    return;
  }
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

  while (ros::ok()){ros::spinOnce();r.sleep();}
  return 0;
}
