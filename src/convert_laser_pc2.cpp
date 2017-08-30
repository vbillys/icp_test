#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
ros::Publisher g_pub_scanpc;

void scanCallback(const sensor_msgs::LaserScanPtr & laser_msg)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.stamp= laser_msg->header.stamp;
  cloud_msg.header.frame_id= laser_msg->header.frame_id;
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  for (size_t i=0;i<laser_msg->ranges.size();i++)
  {
    if (laser_msg->ranges[i] != laser_msg->ranges[i]) continue;
    pcl::PointXYZ point;
    point.x = laser_msg->ranges[i]*cos( laser_msg->angle_min + i*laser_msg->angle_increment);
    point.y = laser_msg->ranges[i]*sin( laser_msg->angle_min + i*laser_msg->angle_increment);
    //point.x = particle_x.at(i);
    //point.y = particle_y.at(i);
    point.z = 0;
    pcl_pc.push_back(point);
  }
  pcl::toROSMsg(pcl_pc, cloud_msg);

  g_pub_scanpc.publish(cloud_msg);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "laser_pc2");
  ros::NodeHandle nh;//("~");
  ros::Subscriber laser_sub = nh.subscribe("/scan", 2, scanCallback);
  //ros::Subscriber map_metadata_sub = nh.subscribe("/map_metadata", 2, mapMetadataCallback);
  g_pub_scanpc = nh.advertise<sensor_msgs::PointCloud2>("/scan_pc2", 3);
  ros::spin();
}

