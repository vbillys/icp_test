#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace std;

ros::Publisher * g_img_synced_pub;
ofstream g_pose_file;
tf::TransformListener * g_tf;

void callback(const CompressedImageConstPtr& image1, const PoseWithCovarianceStampedConstPtr& pose)
{
  static int count = 0;
  tf::StampedTransform t;
  try
  {
  //g_tf->lookupTransform("base_link","map",image1->header.stamp, t);
  g_tf->lookupTransform("map","base_link",image1->header.stamp, t);
  }
  catch(tf2::TransformException& e)
  {
    ROS_INFO("error tf lookup: %s", e.what());
  }
  // 
  // 
  // Solve all of perception here...
  ROS_INFO("%d %d %d %lf %lf", count, image1->header.seq, pose->header.seq, image1->header.stamp.toSec(), pose->header.stamp.toSec());
  ROS_INFO("amcl_pose: %f %f %f  tf_pose: %f %f %f", pose->pose.pose.position.x, pose->pose.pose.position.y, tf::getYaw(pose->pose.pose.orientation),t.getOrigin().getX(), t.getOrigin().getY(), tf::getYaw(t.getRotation()));
  g_img_synced_pub->publish(image1);
  g_pose_file << pose->header.stamp.toSec();
  g_pose_file << ", ";
  g_pose_file << pose->pose.pose.position.x;
  g_pose_file << ", ";
  g_pose_file << pose->pose.pose.position.y;
  g_pose_file << ", ";
  g_pose_file << tf::getYaw(pose->pose.pose.orientation);
  g_pose_file <<endl;
  count++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_pose_img_node");

  ros::NodeHandle nh;

  g_tf = new tf::TransformListener(ros::Duration(5.0));

  g_pose_file.open("pose.txt");
  g_pose_file << fixed;

  ros::Publisher img_synced_pub = nh.advertise<CompressedImage>("/img_sync/compressed",5);
  g_img_synced_pub = & img_synced_pub;

  message_filters::Subscriber<CompressedImage> image1_sub(nh, "/camera/left/image_rect_color/compressed", 1);
  message_filters::Subscriber<PoseWithCovarianceStamped> pose_sub(nh, "/amcl_pose", 1);

  typedef sync_policies::ApproximateTime<CompressedImage, PoseWithCovarianceStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image1_sub, pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  g_pose_file.close();

  return 0;
}
