#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def toM(odom_msg):
    quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
    m = tf.transformations.quaternion_matrix(quaternion)
    m[:3,3] = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
    # print m
    return m

class PoseToTF:
    def __init__(self,  moving_odom_frame, map_frame, odom_topic_name, use_diff_odom):
        rospy.init_node('pose_to_tf', anonymous=False)
        if use_diff_odom:
            rospy.Subscriber(odom_topic_name, Odometry, self.processOdomMsg)
        else:
            rospy.Subscriber(odom_topic_name, Odometry, self.processOdomMsgNoDiff)
        self.pub_br = tf.TransformBroadcaster()
        self.last_odom_msg = Odometry()
        self.moving_odom_frame = moving_odom_frame
        self.map_frame = map_frame
        self.odom_topic_name = odom_topic_name
        self.init = False
        self.use_diff_odom = use_diff_odom

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass


    def processOdomMsgNoDiff(self, msg):
        curr_M = toM(msg)
        pos = curr_M [:3,3]
        quat = tf.transformations.quaternion_from_matrix(curr_M )
        self.pub_br.sendTransform(pos, quat, rospy.Time.now(), self.moving_odom_frame, self.map_frame)

    def processOdomMsg(self, msg):
        if self.init:
            curr_M = toM(msg)
            topub_M =  self.first_odom_M_inv.dot(curr_M)
            # print topub_M
            # topub_M = curr_M * self.first_odom_M_inv
            pos = topub_M[:3,3]
            quat = tf.transformations.quaternion_from_matrix(topub_M)
            self.pub_br.sendTransform(pos, quat, rospy.Time.now(), self.moving_odom_frame, self.map_frame)
        else:
            self.init = True
            self.first_odom_msg = msg
            self.first_odom_M = toM(msg)
            self.first_odom_M_inv = tf.transformations.inverse_matrix(self.first_odom_M)
            # print 'inverse', self.first_odom_M_inv
            self.pub_br.sendTransform((0 , 0 ,0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), self.moving_odom_frame, self.map_frame)

g_moving_odom_frame = 'aft_mapped'
g_map_frame = 'camera_init'
# g_odometry_topic = 'RosAria/pose'
g_odometry_topic = 'aft_mapped_to_init'
g_use_diff_odom = False
if __name__ == '__main__':
    odom = PoseToTF( g_moving_odom_frame, g_map_frame, g_odometry_topic, g_use_diff_odom)
    odom.run()
