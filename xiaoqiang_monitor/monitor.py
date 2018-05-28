#!/usr/bin/env python
"""
The MIT License (MIT)

Copyright (c) 2018 Bluewhale Robot

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author: Xie fusheng
        Randoms
"""


'''
This is a system monitor node for xiaoqiang. Monitor items are power, odom, brightness etc.
System status will be published at report
'''
import rospy
from std_msgs.msg import String, UInt32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from system_monitor.msg import *
import threading
import os
import commands


reportPub = ""
mStatus = Status()
mStatus.brightness = 0
mStatus.imageStatus = False
mStatus.odomStatus = False
mStatus.orbStartStatus = False
mStatus.orbInitStatus = False
mStatus.power = 0
mStatus.orbScaleStatus = False
powerLow = 10.0

mStatusLock = threading.Lock()

def getBrightness(brightness):
    mStatusLock.acquire()
    mStatus.brightness = brightness.data
    mStatusLock.release()

def getImage(image):
    mStatusLock.acquire()
    if image != None:
        mStatus.imageStatus = True
    else:
        mStatus.imageStatus = False
    mStatusLock.release()

def getPower(power):
    mStatusLock.acquire()
    mStatus.power = power.data-0.3
    mStatusLock.release()

def getOdom(odom):
    mStatusLock.acquire()
    if odom != None:
        mStatus.odomStatus = True
    else:
        mStatus.odomStatus = False
    mStatusLock.release()

def getOrbStartStatus(orb_frame):
    mStatusLock.acquire()
    if orb_frame != None:
        mStatus.orbStartStatus = True
    else:
        mStatus.orbStartStatus = False
    mStatusLock.release()

def getOrbTrackingFlag(cam_pose):
    mStatusLock.acquire()
    if cam_pose != None:
        mStatus.orbInitStatus = True
    else:
        mStatus.orbInitStatus = False
    mStatusLock.release()

def getOrbScaleStatus(flag):
    mStatusLock.acquire()
    mStatus.orbScaleStatus = flag.data
    mStatusLock.release()


def monitor():
    global reportPub
    rospy.init_node("monitor", anonymous=True)
    rospy.Subscriber("/usb_cam/brightness", UInt32, getBrightness)
    rospy.Subscriber("/usb_cam/image_raw", Image, getImage)
    rospy.Subscriber("/orb_slam/frame", Image, getOrbStartStatus)
    rospy.Subscriber("/xiaoqiang_driver/power", Float64, getPower)
    rospy.Subscriber("/xiaoqiang_driver/odom", Odometry, getOdom)
    rospy.Subscriber("/orb_slam/camera", rospy.msg.AnyMsg, getOrbTrackingFlag)
    rospy.Subscriber("/orb_scale/scale_status", Bool, getOrbScaleStatus)
    reportPub = rospy.Publisher('/system_monitor/report', Status , queue_size
=0)


if __name__ == "__main__":
    monitor()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        mStatusLock.acquire()
        if reportPub !=  "":
            reportPub.publish(mStatus)
        # clear data
        mStatus.brightness = 0
        mStatus.power = 0
        mStatus.imageStatus = False
        mStatus.odomStatus = False
        mStatus.orbInitStatus = False
        mStatus.orbStartStatus = False
        mStatus.orbScaleStatus = False
        mStatusLock.release()
        rate.sleep()
