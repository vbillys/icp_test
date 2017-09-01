#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
ros::Publisher g_pub_laserscan;
void odomCallback(const nav_msgs::Odometry & odom_msg)
{
  static tf::TransformBroadcaster tf_br;
  tf::Transform transform_content;
  //tf::poseMsgToTF(odom_msg, transform_content);
  transform_content.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z));
  tf::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
  //q.setRPY(0, 0, tf::getYaw(transform.getRotation()));
  transform_content.setRotation(q);
  tf_br.sendTransform(tf::StampedTransform(transform_content, ros::Time::now(), "map_odom","base_link_odom" ));
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "repub_odom_tf_node");
  ros::NodeHandle nh;//("~");
  ros::Subscriber laser_sub = nh.subscribe("/RosAria/pose", 2, odomCallback);
  //ros::Subscriber map_metadata_sub = nh.subscribe("/map_metadata", 2, mapMetadataCallback);
  //g_pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/scan_recalc", 3);
  ros::spin();
}
   
