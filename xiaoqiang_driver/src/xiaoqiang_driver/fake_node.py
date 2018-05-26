#!/usr/bin/env python
#encoding=utf-8

"""
这是一个小强驱动包的假节点。这个节点发布以下几个topic
odom
status_flag
pose2d
power
位置数据全部为0， 电压数据12V
同时发布odom -> baselink 的 静态tf变换
"""

import rospy
from std_msgs.msg import Int32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


if __name__ == "__main__":
    rospy.init_node("fake_xiaoqiang_driver", anonymous=True)
    rate = rospy.Rate(50)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=0)
    status_flag_pub = rospy.Publisher("status_flag", Int32,
        queue_size=0)
    pose_pub = rospy.Publisher("pose2d", Pose2D, queue_size=0)
    power_pub = rospy.Publisher("power", Float64, queue_size=0)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        mOdom = Odometry()
        mOdom.header.stamp = now
        mOdom.header.frame_id = "odom"
        mOdom.child_frame_id = "base_footprint"
        mOdom.pose.pose.orientation.x = 0
        mOdom.pose.pose.orientation.y = 0
        mOdom.pose.pose.orientation.z = 0
        mOdom.pose.pose.orientation.w = 1
        odom_pub.publish(mOdom)
        status = Int32()
        status.data = 1
        status_flag_pub.publish(status)
        mPose = Pose2D()
        pose_pub.publish(mPose)
        mPower = Float64()
        mPower.data = 12.0
        power_pub.publish(mPower)
        rate.sleep()
