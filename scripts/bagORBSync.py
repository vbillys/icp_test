#! /usr/bin/python
import rosbag
import rospy
import sys
import time
import tf
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo
print sys.argv[1]
print sys.argv[2]
bag = rosbag.Bag(sys.argv[1])
f = open(sys.argv[2], "r")
lines = f.readlines()
print bag
left_image_topic = '/camera/left/image_rect_color/compressed'
right_image_topic = '/camera/right/image_rect_color/compressed'
left_camera_info_topic = '/camera/left/camera_info'
right_camera_info_topic = '/camera/right/camera_info'
# bag_messages = bag.read_messages(topics=['/scan', left_image_topic , right_image_topic, left_image_topic, left_camera_info_topic])
# print lines
print 'processing trajectory...'
clock = Clock()
clock_msg = Time()
rospy.init_node('recordedFrameSyncPublisher', anonymous=False)
pub_clock   = rospy.Publisher('/clock', Clock)
pub_scan    = rospy.Publisher('/scan',  LaserScan)
pub_right_image = rospy.Publisher(right_image_topic, CompressedImage)
pub_left_image = rospy.Publisher(left_image_topic, CompressedImage)
pub_right_camera_info = rospy.Publisher(right_camera_info_topic, CameraInfo)
pub_left_camera_info = rospy.Publisher(left_camera_info_topic, CameraInfo)
pub_br = tf.TransformBroadcaster()
# for topic, msg, t in bag.read_messages(topics=['/scan', '/odom_pose_do_not_subscribe', '/tf']):
#first message
# (topic,msg,t) = bag_messages.next()
# clock.clock = msg.header.stamp
# clock_msg.data = msg.header.stamp
count_msgs  = 0
count_frame  = 0#2267#0#2267#0#2267#0
count_frame  = int(sys.argv[3])#0#9552#6269#2763#2267#0#2267#0#2267#0
sleep_period = float(sys.argv[4])
# print t
words = lines[count_frame].split()
# print words
ref_clock = rospy.Time.from_sec(float(words[0]))
# print ref_clock.to_sec()
# prepare for caching messages
left_camera_info_msg = CameraInfo()
right_camera_info_msg = CameraInfo()
left_image_msg = CompressedImage()
right_image_msg = CompressedImage()
# for topic, msg, t in bag.read_messages(topics=['/scan']):
for topic, msg, t in bag.read_messages(topics=['/scan', left_image_topic , right_image_topic, right_camera_info_topic, left_camera_info_topic]):
# for line in lines:
    if rospy.is_shutdown():
        break
    # clocking triggered by checking scan topic only
    # print topic
    if topic != '/scan':
        #here we just cache incoming message but not publish
        if topic == left_image_topic:
            # print 'got left image'
            left_image_msg = msg
        elif topic == right_image_topic:
            # print 'got right image'
            right_image_msg = msg
        elif topic == left_camera_info_topic:
            # print 'got left camera_info'
            left_camera_info_msg = msg
        elif topic == right_camera_info_topic:
            # print 'got right camera_info'
            right_camera_info_msg = msg
        pass
    else:
        if ref_clock > t :#+ rospy.Duration.from_sec(0.5):
            count_msgs = count_msgs + 1
            # message = bag_messages.next()
            # # publish clock only
            # clock.clock = msg.header.stamp
            # pub_clock.publish(clock)
        else:
            print t.to_sec(),count_msgs, ref_clock.to_sec(), count_frame
            #publish message
            # clock.clock = msg.header.stamp
            # clock_msg.data = msg.header.stamp
            clock.clock= ref_clock
            left_image_msg.header.stamp = ref_clock
            right_image_msg.header.stamp = ref_clock
            left_camera_info_msg.header.stamp  = ref_clock
            right_camera_info_msg.header.stamp = ref_clock
            pub_clock.publish(clock)
            pub_scan.publish(msg)
            pub_left_image.publish(left_image_msg)
            pub_right_image.publish(right_image_msg)
            pub_left_camera_info.publish(left_camera_info_msg)
            pub_right_camera_info.publish(right_camera_info_msg)
            # pub_br.sendTransform((float(words[1]) ,float(words[2]) ,float(words[3])  ), (float(words[4]) ,float(words[5]) ,float(words[6]) ,float(words[7])), rospy.Time.now(), "base_link", "odom")
            print (float(words[3]) ,-float(words[1]) ,-float(words[2])  )
            pub_br.sendTransform((float(words[3]) ,-float(words[1]) ,-float(words[2])  ), (float(words[6]) ,-float(words[4]) ,-float(words[5]) ,float(words[7])), rospy.Time.now(), "base_link", "odom")
            #get next frame
            count_frame = count_frame + 1
            words = lines[count_frame].split()
            ref_clock = rospy.Time.from_sec(float(words[0]))
            time.sleep(sleep_period) #(0.018)
        # for topic, msg, t in bag.read_messages(topics=['/scan']):
            # print topic
            # time.sleep(0.5)
            # if topic != '/tf':
                # print msg.header.stamp
                # clock.clock = msg.header.stamp
                # pub_clock.publish(clock)
                # pub_scan.publish(msg)
            # else:
                # print msg.transforms[0].header.stamp
                # clock.clock = msg.transforms[0].header.stamp
                # pub_clock.publish(clock)
bag.close()
f.close()
