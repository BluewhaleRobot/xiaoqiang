#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Xie fusheng
#         Randoms


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
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int32

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
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.x = 0
        odom.pose.pose.orientation.y = 0
        odom.pose.pose.orientation.z = 0
        odom.pose.pose.orientation.w = 1
        odom_pub.publish(odom)
        status = Int32()
        status.data = 1
        status_flag_pub.publish(status)
        mPose = Pose2D()
        pose_pub.publish(mPose)
        power = Float64()
        power.data = 12.0
        power_pub.publish(power)
        rate.sleep()
