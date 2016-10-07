#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
ros::Publisher g_pub_laserscan;
void scanCallback(const sensor_msgs::LaserScanPtr & laser_msg)
{
  tf::StampedTransform transform;
  static tf::TransformListener tf_listener;
  static tf::TransformBroadcaster tf_br;
            try
            {
                tf_listener.lookupTransform("map","base_link",ros::Time(0), transform);
              //pose_from_tf.x = transform.getOrigin().x();
              //pose_from_tf.y = transform.getOrigin().y();
              //pose_from_tf.theta = tf::getYaw(transform.getRotation());
              //ROS_INFO_STREAM(pose_from_tf);
  tf::Transform transform_content;
  transform_content.setOrigin(tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(),0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, tf::getYaw(transform.getRotation()));
  transform_content.setRotation(q);
  //tf_br.sendTransform(tf::StampedTransform(transform_content, ros::Time::now(), "odom","base_link_recalc" ));
  tf_br.sendTransform(tf::StampedTransform(transform_content, ros::Time::now(), "odom_recalc","base_link_recalc" ));
  //sensor_msgs::LaserScanPtr scan_msg_recalc = sensor_msgs::LaserScanPtr(new sensor_msgs::LaserScan(*laser_msg));
  //(*scan_msg_recalc).frame_id = "laser_recalc";
  sensor_msgs::LaserScan scan_msg_recalc = *laser_msg;
  scan_msg_recalc.header.frame_id = "laser_recalc";
  g_pub_laserscan.publish(scan_msg_recalc);
            }
            catch (tf::TransformException ex)
            {
              ROS_WARN("%s",ex.what());
            }
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "user_mapping_node");
  ros::NodeHandle nh;//("~");
  ros::Subscriber laser_sub = nh.subscribe("/scan", 2, scanCallback);
  //ros::Subscriber map_metadata_sub = nh.subscribe("/map_metadata", 2, mapMetadataCallback);
  g_pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/scan_recalc", 3);
  ros::spin();
}
   
