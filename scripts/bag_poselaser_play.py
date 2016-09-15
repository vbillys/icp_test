#! /usr/bin/python
import rosbag
import rospy
import sys
from sensor_msgs.msg import LaserScan
from ros_graphslam.msg import pose_laser
import time
import tf

bag = rosbag.Bag(sys.argv[1])
sleep_period = float(sys.argv[2])

rospy.init_node('recordedFrameSyncPublisher', anonymous=False)
pub_scan    = rospy.Publisher('/scan',  LaserScan, queue_size=5)
br = tf.TransformBroadcaster()
for topic, msg, t in bag.read_messages(topics=['pose_laser']):
    time.sleep(sleep_period) #(0.018)
    if rospy.is_shutdown():
        break
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y,0),
                     (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'base_link','odom')
    br.sendTransform((0.2,0,0),tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),'laser','base_link')
    msg_laser = msg.scan
    msg_laser.header.frame_id = 'laser'
    msg_laser.header.stamp = rospy.Time.now()
    pub_scan.publish(msg_laser)
bag.close()
