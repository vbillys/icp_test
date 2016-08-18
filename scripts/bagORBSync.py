#! /usr/bin/python
import rosbag
import rospy
import sys
import time
import tf
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Time
from sensor_msgs.msg import LaserScan
print sys.argv[1]
print sys.argv[2]
bag = rosbag.Bag(sys.argv[1])
f = open(sys.argv[2], "r")
lines = f.readlines()
print bag
bag_messages = bag.read_messages(topics=['/scan'])
# print lines
print 'processing trajectory...'
clock = Clock()
clock_msg = Time()
rospy.init_node('recordedFrameSyncPublisher', anonymous=False)
pub_clock   = rospy.Publisher('/clock', Clock)
pub_scan    = rospy.Publisher('/scan',  LaserScan)
pub_br = tf.TransformBroadcaster()
# for topic, msg, t in bag.read_messages(topics=['/scan', '/odom_pose_do_not_subscribe', '/tf']):
#first message
# (topic,msg,t) = bag_messages.next()
# clock.clock = msg.header.stamp
# clock_msg.data = msg.header.stamp
count_msgs  = 0
count_frame  = 0#2267#0#2267#0#2267#0
count_frame  = 9552#6269#2763#2267#0#2267#0#2267#0
# print t
words = lines[count_frame].split()
# print words
ref_clock = rospy.Time.from_sec(float(words[0]))
# print ref_clock.to_sec()
for topic, msg, t in bag.read_messages(topics=['/scan']):
# for line in lines:
    if rospy.is_shutdown():
        break
    if ref_clock > t + rospy.Duration.from_sec(0.5):
        count_msgs = count_msgs + 1
        # message = bag_messages.next()
        # publish clock only
        clock.clock = msg.header.stamp
        pub_clock.publish(clock)
    else:
        print t.to_sec(),count_msgs, ref_clock.to_sec(), count_frame
        #publish message
        clock.clock = msg.header.stamp
        clock_msg.data = msg.header.stamp
        pub_clock.publish(clock)
        pub_scan.publish(msg)
        # pub_br.sendTransform((float(words[1]) ,float(words[2]) ,float(words[3])  ), (float(words[4]) ,float(words[5]) ,float(words[6]) ,float(words[7])), rospy.Time.now(), "base_link", "odom")
        print (float(words[3]) ,-float(words[1]) ,-float(words[2])  )
        pub_br.sendTransform((float(words[3]) ,-float(words[1]) ,-float(words[2])  ), (float(words[6]) ,-float(words[4]) ,-float(words[5]) ,float(words[7])), rospy.Time.now(), "base_link", "odom")
        #get next frame
        count_frame = count_frame + 1
        words = lines[count_frame].split()
        ref_clock = rospy.Time.from_sec(float(words[0]))
    time.sleep(0.08)
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
