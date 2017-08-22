#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
ros::Publisher g_pub_laserscan;
void scanCallback(const sensor_msgs::LaserScanPtr & laser_msg)
{
  tf::StampedTransform transform, laser_transform;
  static tf::TransformListener tf_listener;
  static tf::TransformBroadcaster tf_br;
	    try
	    {
		tf_listener.lookupTransform("odom","base_link",ros::Time(0), transform);
		tf_listener.lookupTransform("base_link","laser",ros::Time(0), laser_transform);
	      //pose_from_tf.x = transform.getOrigin().x();
	      //pose_from_tf.y = transform.getOrigin().y();
	      //pose_from_tf.theta = tf::getYaw(transform.getRotation());
	      //ROS_INFO_STREAM(pose_from_tf);
  tf::Transform transform_content, transform_laser;
  transform_content.setOrigin(tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(),0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, tf::getYaw(transform.getRotation()));
  transform_content.setRotation(q);
  transform_laser.setOrigin(tf::Vector3(laser_transform.getOrigin().x(), laser_transform.getOrigin().y(),laser_transform.getOrigin().z()));
  q.setRPY(0, 0, tf::getYaw(laser_transform.getRotation()));
  transform_laser.setRotation(q);
  //tf_br.sendTransform(tf::StampedTransform(transform_content, ros::Time::now(), "odom","base_link_recalc" ));
  tf_br.sendTransform(tf::StampedTransform(transform_content, ros::Time::now(), "odom_qmcl","base_link_qmcl" ));
  tf_br.sendTransform(tf::StampedTransform(transform_laser, ros::Time::now(), "base_link_qmcl", "laser_qmcl" ));
  //sensor_msgs::LaserScanPtr scan_msg_recalc = sensor_msgs::LaserScanPtr(new sensor_msgs::LaserScan(*laser_msg));
  //(*scan_msg_recalc).frame_id = "laser_recalc";
  sensor_msgs::LaserScan scan_msg_recalc = *laser_msg;
  scan_msg_recalc.header.frame_id = "laser_qmcl";
  g_pub_laserscan.publish(scan_msg_recalc);
	    }
	    catch (tf::TransformException ex)
	    {
	      ROS_WARN("%s",ex.what());
	    }
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "repub_laser");
  ros::NodeHandle nh;//("~");
  ros::Subscriber laser_sub = nh.subscribe("/scan", 2, scanCallback);
  //ros::Subscriber map_metadata_sub = nh.subscribe("/map_metadata", 2, mapMetadataCallback);
  g_pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/scan_qmcl", 3);
  ros::spin();
}
   
