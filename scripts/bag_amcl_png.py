#! /usr/bin/python
import rosbag
import rospy
import sys
import time
import tf
import math
import copy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo

sleep_period = float(sys.argv[2])
bag = rosbag.Bag(sys.argv[1])
left_image_topic = '/camera/left/image_rect_color/compressed'
left_caminfo = '/camera/left/camera_info'
amcl_pose_topic  = '/amcl_pose'
sync_image_topic = '/img_sync/compressed'

rospy.init_node('recordedAmclCompressedImgSyncPublisher', anonymous=False)
pub_image = rospy.Publisher(sync_image_topic, CompressedImage)
pub_caminfo = rospy.Publisher(left_caminfo, CameraInfo)

f_handle_w = open('pose.txt','w')

count = 0
last_image = CompressedImage()

def writePoseStamped(time_stamp, x, y, yaw):
    line_str = format(time_stamp,'.9f') + ', '
    line_str = line_str + format(x,'.6f') + ', '
    line_str = line_str + format(y,'.6f') + ', '
    line_str = line_str + format(yaw,'.6f') + '\n'
    f_handle_w.write(line_str)

def publishImgAndSavePose(msg, last_image, sleep_period):
    time.sleep(sleep_period)
    pub_image.publish(last_image)    
    pub_caminfo.publish(CameraInfo())
    quaternion = ( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) 
    euler = tf.transformations.euler_from_quaternion(quaternion)
    writePoseStamped(msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

first_img_arrived = False
for topic, msg, t in bag.read_messages(topics=[ left_image_topic ,  amcl_pose_topic, left_caminfo]):
    if rospy.is_shutdown():
        break
    # if topic == left_caminfo:
    if topic == left_image_topic:
        last_image = copy.deepcopy(msg)
        first_img_arrived = True
    if topic == amcl_pose_topic and first_img_arrived:
        publishImgAndSavePose(msg, last_image, sleep_period)
        print count
        count = count + 1
bag.close()
f_handle_w.close()
