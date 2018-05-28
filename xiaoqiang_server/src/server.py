#!/usr/bin/env python
# coding:utf-8
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




import commands
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from socket import (AF_INET, SO_BROADCAST, SOCK_DGRAM, SOL_SOCKET, socket,
                    timeout)

import numpy as np
import psutil
import rospy
import tf
from geometry_msgs.msg import Pose, Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, Int16, String, UInt32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from xiaoqiang_msgs.msg import Status

HOST = ''  # should not be 127.0.0.1 or localhost
USER_SOCKET_PORT = 20001  # 局域网udp命令监听端口
USER_SOCKET_REMOTE = None
USER_SERVER_SOCKET = None
BUFSIZE = 1024

PACKAGE_HEADER = [205, 235, 215]
DATA_CACHE = []

CMD_PUB = None
MAP_SAVE_PUB = None

MAX_VEL = 0.8
MAX_THETA = 3.6
STATUS = Status()
STATUS.brightness = 0.0
STATUS.image_status = False
STATUS.odom_status = False
STATUS.orb_start_status = False
STATUS.orb_init_status = False
STATUS.power = 0.0
STATUS.orb_scale_status = False
POWER_LOW = 10.0

STATUS_LOCK = threading.Lock()

DATA_CACHE = []
CURRENT_POSE = Pose()
SEND_DATA = bytearray([205, 235, 215, 24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
SPEED_CMD = Twist()

MAP_THREAD = None
NAV_THREAD = None
CONTROL_FLAG = False

SCALE_ORB_THREAD = None
GLOBAL_MOVE_PUB = None
EVEVATOR_PUB = None

TILT_PUB = None
MAV_LAST_TIME = None


def unpack_req(req):
    global DATA_CACHE
    res = []
    package_list = split_req(req)
    # process the first package
    completeData = DATA_CACHE + package_list[0]
    package_list.remove(package_list[0])
    package_list = split_req(completeData) + package_list

    for count in range(0, len(package_list)):
        if len(package_list[count]) != 0 and len(package_list[count]) == package_list[count][0] + 1:
            res.append(package_list[count][1:])
    last_one = package_list[-1:][0]  # the last one
    if len(last_one) == 0 or len(last_one) != last_one[0] + 1:
        DATA_CACHE = last_one
    return res


def find_package_header(req):
    if len(req) < 3:
        return -1
    for count in range(0, len(req) - 2):
        if req[count] == PACKAGE_HEADER[0] and req[count + 1] == PACKAGE_HEADER[1] and req[count + 2] == PACKAGE_HEADER[2]:
            return count
    return -1


def split_req(req):
    res = []
    start_index = 0
    new_index = 0
    while True:
        new_index = find_package_header(req[start_index:])
        if new_index == -1:
            break
        res.append(req[start_index: start_index + new_index])
        start_index = new_index + 3 + start_index
    res.append(req[start_index:])
    return res


def parse_data(cmds):
    global CONTROL_FLAG
    res = None
    for count in range(0, len(cmds)):
        if len(cmds[count]) > 0:
            CONTROL_FLAG = True
        # 判断是否为关机命令
        if len(cmds[count]) == 2:
            global_move_flag = Bool()
            global_move_flag.data = True

            if cmds[count][0] == 0xaa and cmds[count][1] == 0x44:
                rospy.loginfo("system poweroff")
                commands.getstatusoutput(
                    '/sbin/shutdown -h now')

            if cmds[count][0] == ord('f'):
                rospy.loginfo("forward")
                GLOBAL_MOVE_PUB.publish(global_move_flag)
                SPEED_CMD.linear.x = MAX_VEL*cmds[count][1]/100.0
                CMD_PUB.publish(SPEED_CMD)
            elif cmds[count][0] == ord('b'):
                rospy.loginfo("back")
                GLOBAL_MOVE_PUB.publish(global_move_flag)
                SPEED_CMD.linear.x = -MAX_VEL*cmds[count][1]/100.0
                CMD_PUB.publish(SPEED_CMD)
            elif cmds[count][0] == ord('c'):
                rospy.loginfo("circleleft")
                GLOBAL_MOVE_PUB.publish(global_move_flag)
                SPEED_CMD.angular.z = MAX_THETA*cmds[count][1]/100.0/2.8
                CMD_PUB.publish(SPEED_CMD)
            elif cmds[count][0] == ord('d'):
                rospy.loginfo("circleright")
                GLOBAL_MOVE_PUB.publish(global_move_flag)
                SPEED_CMD.angular.z = -MAX_THETA*cmds[count][1]/100.0/2.8
                CMD_PUB.publish(SPEED_CMD)
            elif cmds[count][0] == ord('s'):
                rospy.loginfo("stop")
                SPEED_CMD.linear.x = 0
                SPEED_CMD.angular.z = 0
                CMD_PUB.publish(SPEED_CMD)
    return res


class UserServer(threading.Thread):
    # 接收udp命令的socket
    def __init__(self):
        global USER_SERVER_SOCKET
        super(UserServer, self).__init__()
        self._stop = threading.Event()
        USER_SERVER_SOCKET = socket(AF_INET, SOCK_DGRAM)
        USER_SERVER_SOCKET.bind((HOST, USER_SOCKET_PORT))

    def stop(self):
        if USER_SERVER_SOCKET != None:
            USER_SERVER_SOCKET.close()
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global USER_SOCKET_REMOTE
        USER_SERVER_SOCKET.settimeout(2)  # 设置udp 2秒超时 等待
        while not self.stopped() and not rospy.is_shutdown():
            try:
                data, USER_SOCKET_REMOTE = USER_SERVER_SOCKET.recvfrom(BUFSIZE)
            except timeout:
                continue
            if not data:
                break
            dataList = []
            for c in data:
                dataList.append(ord(c))
            parse_data(unpack_req(dataList))  # 处理命令数据
        self.stop()


def get_power(power):
    STATUS_LOCK.acquire()
    STATUS.power = power.data
    STATUS_LOCK.release()


def get_image(image):
    STATUS_LOCK.acquire()
    if image != None:
        STATUS.image_status = True
    else:
        STATUS.image_status = False
    STATUS_LOCK.release()


def get_odom(odom):
    global CURRENT_POSE
    STATUS_LOCK.acquire()
    if odom != None:
        STATUS.odom_status = True
        CURRENT_POSE = odom.pose.pose  # 更新坐标

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


def get_global_move_flag(move_enable):
    if not move_enable.data:
        # 关闭视觉导航
        if not NAV_THREAD.stopped():
            NAV_THREAD.stop()


def broadcast():
    global CMD_PUB, GLOBAL_MOVE_PUB, MAV_LAST_TIME
    rospy.init_node("broadcast", anonymous=True)
    MAV_LAST_TIME = rospy.Time.now()
    rospy.Subscriber("/xiaoqiang_driver/power", Float64, get_power)
    rospy.Subscriber("/usb_cam/image_raw", Image, get_image)
    rospy.Subscriber("/xiaoqiang_driver/odom", Odometry, get_odom)
    rospy.Subscriber("/orb_slam/camera", rospy.msg.AnyMsg, get_orb_tracking_flag)
    rospy.Subscriber("/orb_slam/frame", Image, get_orb_start_status)
    rospy.Subscriber("/xiaoqiang_driver/global_move_flag",
                     Bool, get_global_move_flag)
    GLOBAL_MOVE_PUB = rospy.Publisher("/xiaoqiang_driver/global_move_flag", Bool, queue_size=1)
    CMD_PUB = rospy.Publisher('/xiaoqiang_driver/cmd_vel', Twist, queue_size=0)


if __name__ == "__main__":
    broadcast()
    rate = rospy.Rate(10)
    # 配置udp广播
    MYPORT = 22001  # 广播端口
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    # 开启udp接收监听线程
    user_server_thread = UserServer()
    user_server_thread.start()
    i = 10
    ii = 40
    cmd4 = "aplay /home/xiaoqiang/Desktop/d.wav"
    while not rospy.is_shutdown():
        # #每２秒提示一下
        # if ii==10:
        #     subprocess.Popen(cmd4,shell=True)

        # if not control_flag and ii>32 and  ii<37:
        #     speed_cmd.linear.x = 0
        #     speed_cmd.angular.z = 0
        #     cmd_pub.publish(speed_cmd)
        # 每4秒心跳维护一次
        if ii == 40:
            ii = 0
            CONTROL_FLAG = False
        ii += 1
        # 持续反馈状态
        if USER_SOCKET_REMOTE != None and USER_SERVER_SOCKET != None:

            SEND_DATA[4:8] = map(ord, struct.pack('f', 0.0))
            SEND_DATA[8:12] = map(ord, struct.pack('f', 0.0))
            SEND_DATA[12:16] = map(ord, struct.pack('f', 0.0))
            SEND_DATA[16:20] = map(ord, struct.pack('f', STATUS.power))
            SEND_DATA[24:28] = map(ord, struct.pack('f', 0.0))
            if STATUS.odom_status:
                statu0 = 0x01  # 混合里程计
            else:
                statu0 = 0x00
            if STATUS.image_status:
                statu1 = 0x02  # 视觉摄像头
            else:
                statu1 = 0x00
            if STATUS.orb_start_status:
                statu2 = 0x04  # 视觉系统状态
            else:
                statu2 = 0x00
            if STATUS.orb_init_status:
                statu3 = 0x08  # 视觉系统状态
            else:
                statu3 = 0x00
            SEND_DATA[20] = statu0 + statu1 + statu2 + statu3
            try:
                USER_SERVER_SOCKET.sendto(bytes(SEND_DATA), USER_SOCKET_REMOTE)
            except:
                print "remote disconnect !\n"

        # 每秒广播一次
        if i == 10:
            i = 0
            data = "xq"
            # 发送广播包
            try:
                s.sendto(data, ('<broadcast>', MYPORT))
            except:
                continue
            # clear data
            STATUS.power = 0.0
            STATUS.orb_init_status = False
            STATUS.orb_start_status = False
            STATUS.image_status = False
            STATUS.odom_status = False
        i += 1
        rate.sleep()
    user_server_thread.stop()
