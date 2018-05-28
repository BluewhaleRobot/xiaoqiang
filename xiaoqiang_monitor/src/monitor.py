#!/usr/bin/env python
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


'''
This is a system monitor node for xiaoqiang. Monitor items are power, odom, brightness etc.
System status will be published at report
'''
import commands
import os
import threading

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String, UInt32
from xiaoqiang_msgs.msg import Status

REPORT_PUB = ""
STATUS = Status()
STATUS.brightness = 0
STATUS.image_status = False
STATUS.odom_status = False
STATUS.orb_start_status = False
STATUS.orb_init_status = False
STATUS.power = 0
STATUS.orb_scale_status = False
POWER_LOW = 10.0

STATUS_LOCK = threading.Lock()


def get_brightness(brightness):
    STATUS_LOCK.acquire()
    STATUS.brightness = brightness.data
    STATUS_LOCK.release()


def get_image(image):
    STATUS_LOCK.acquire()
    if image != None:
        STATUS.image_status = True
    else:
        STATUS.image_status = False
    STATUS_LOCK.release()


def get_power(power):
    STATUS_LOCK.acquire()
    STATUS.power = power.data-0.3
    STATUS_LOCK.release()


def get_odom(odom):
    STATUS_LOCK.acquire()
    if odom != None:
        STATUS.odom_status = True
    else:
        STATUS.odom_status = False
    STATUS_LOCK.release()


def get_orb_start_status(orb_frame):
    STATUS_LOCK.acquire()
    if orb_frame != None:
        STATUS.orb_start_status = True
    else:
        STATUS.orb_start_status = False
    STATUS_LOCK.release()


def get_orb_tracking_flag(camera_pose):
    STATUS_LOCK.acquire()
    if camera_pose != None:
        STATUS.orb_init_status = True
    else:
        STATUS.orb_init_status = False
    STATUS_LOCK.release()


def get_orb_scale_status(flag):
    STATUS_LOCK.acquire()
    STATUS.orb_scale_status = flag.data
    STATUS_LOCK.release()


def monitor():
    global REPORT_PUB
    rospy.init_node("monitor", anonymous=True)
    rospy.Subscriber("/camera_node/brightness", UInt32, get_brightness)
    rospy.Subscriber("/camera_node/image_raw", Image, get_image)
    rospy.Subscriber("/orb_slam/frame", Image, get_orb_start_status)
    rospy.Subscriber("/xiaoqiang_driver/power", Float64, get_power)
    rospy.Subscriber("/xiaoqiang_driver/odom", Odometry, get_odom)
    rospy.Subscriber("/orb_slam/camera", rospy.msg.AnyMsg,
                     get_orb_tracking_flag)
    rospy.Subscriber("/orb_slam/scale_status", Bool, get_orb_scale_status)
    REPORT_PUB = rospy.Publisher(
        '/xiaoqiang_monitor/report', Status, queue_size=0)


if __name__ == "__main__":
    monitor()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        STATUS_LOCK.acquire()
        if REPORT_PUB != "":
            REPORT_PUB.publish(STATUS)
        # clear data
        STATUS.brightness = 0
        STATUS.power = 0
        STATUS.image_status = False
        STATUS.odom_status = False
        STATUS.orb_init_status = False
        STATUS.orb_start_status = False
        STATUS.orb_scale_status = False
        STATUS_LOCK.release()
        rate.sleep()
