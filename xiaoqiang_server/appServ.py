#!/usr/bin/env python
#coding:utf-8

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool,Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Pose
from sensor_msgs.msg import Image
from system_monitor.msg import *
import threading
import os
import sys
from socket import *
import commands
import struct
from geometry_msgs.msg import Twist
import time,psutil,subprocess,signal
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as  np

HOST = ''#should not be 127.0.0.1 or localhost
UserSocket_port=20001  #局域网udp命令监听端口
UserSocket_remote=None
UserSerSocket=None;
BUFSIZE = 1024

PACKAGE_HEADER = [205, 235, 215]
dataCache = []

cmd_pub=None
mapSave_pub=None

maxVel=0.8
maxTheta=3.6
mStatus = Status()
mStatus.brightness = 0.0
mStatus.imageStatus = False
mStatus.odomStatus = False
mStatus.orbStartStatus = False
mStatus.orbInitStatus = False
mStatus.power = 0.0
mStatus.orbScaleStatus = False
powerLow = 10.0

mStatusLock = threading.Lock()

dataCache = []
currentPose=Pose();
sendData=bytearray([205,235,215,24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
speed_cmd=Twist()

mapthread=None
navthread=None
control_flag=False

scaleOrbThread=None
globalMovePub=None
elevatorPub=None

navFlag=False
tilt_pub=None
nav_lastTime=None
def getDataFromReq(req):
    pass

def unpackReq(req):
    global dataCache
    res = []
    packageList = splitReq(req)
    # process the first package
    completeData = dataCache + packageList[0]
    packageList.remove(packageList[0])
    packageList =  splitReq(completeData) + packageList

    for count in range(0, len(packageList)):
        if len(packageList[count]) != 0 and len(packageList[count]) == packageList[count][0] + 1:
            res.append(packageList[count][1:])
    lastOne = packageList[-1:][0] # the last one
    if len(lastOne) == 0 or len(lastOne) != lastOne[0] + 1:
        dataCache = lastOne
    return res


def findPackageHeader(req):
    if len(req) < 3:
        return -1
    for count in range(0, len(req) - 2):
        if req[count] == PACKAGE_HEADER[0] and req[count + 1] == PACKAGE_HEADER[1] and req[count + 2] == PACKAGE_HEADER[2]:
            return count
    return -1

def splitReq(req):
    res = []
    startIndex = 0
    newIndex = 0
    while True:
        newIndex = findPackageHeader(req[startIndex:])
        if newIndex == -1:
            break
        res.append(req[startIndex: startIndex + newIndex])
        startIndex = newIndex + 3 + startIndex
    res.append(req[startIndex:])
    return res

def parseData(cmds):
    global cmd_pub, maxVel, maxTheta,mapthread,speed_cmd,control_flag
    global scaleOrbThread,globalMovePub,elevatorPub
    global navFlag,tilt_pub,mapSave_pub,nav_lastTime
    res = None
    time_now=rospy.Time.now()
    for count in range(0, len(cmds)):
        if len(cmds[count])>0:
            control_flag=True
        #判断是否为关机命令
        if len(cmds[count])==2:
            globalMoveFlag=Bool()
            globalMoveFlag.data=True

            if cmds[count][0]==0xaa and cmds[count][1]==0x44:
                print "system poweroff"
                status, output = commands.getstatusoutput('/sbin/shutdown -h now')

            if cmds[count][0]==ord('f'):
                print "forward"
                globalMovePub.publish(globalMoveFlag)
                speed_cmd.linear.x=maxVel*cmds[count][1]/100.0
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('b'):
                print "back"
                globalMovePub.publish(globalMoveFlag)
                speed_cmd.linear.x=-maxVel*cmds[count][1]/100.0
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('c'):
                print "circleleft"
                globalMovePub.publish(globalMoveFlag)
                speed_cmd.angular.z=maxTheta*cmds[count][1]/100.0/2.8
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('d'):
                print "circleright"
                globalMovePub.publish(globalMoveFlag)
                speed_cmd.angular.z=-maxTheta*cmds[count][1]/100.0/2.8
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('s'):
                print "stop"
                speed_cmd.linear.x = 0
                speed_cmd.angular.z = 0
                cmd_pub.publish(speed_cmd)
        #only for debug
        # print "recive orders"+str(cmds[count])
    return res

class UserSer(threading.Thread):
    #接收udp命令的socket
    def __init__(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        super(UserSer, self).__init__()
        self._stop = threading.Event()
        UserSerSocket=socket(AF_INET, SOCK_DGRAM)
        UserSerSocket.bind((HOST,UserSocket_port))
    def stop(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        if UserSerSocket!=None:
            UserSerSocket.close()
        self._stop.set()

    def stopped(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        return self._stop.isSet()

    def run(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        UserSerSocket.settimeout(2) #设置udp 2秒超时 等待
        while not self.stopped() and not rospy.is_shutdown():
            try:
                data, UserSocket_remote = UserSerSocket.recvfrom(BUFSIZE)
            except timeout:
                # print "timeout"
                continue
            #print "UserSocket get data"
            if not data: break
            #rospy.loginfo(data)
            dataList = []
            for c in data:
                dataList.append(ord(c))
            parseData(unpackReq(dataList))  ##处理命令数据
        self.stop();



def getPower(power):
    mStatusLock.acquire()
    mStatus.power = power.data
    #if power.data < powerLow and not os.path.isfile(powerFlagFilePath) and power.data > 0.1:
    mStatusLock.release()

def getImage(image):
    mStatusLock.acquire()
    if image != None:
        mStatus.imageStatus = True
    else:
        mStatus.imageStatus = False
    mStatusLock.release()

def getOdom(odom):
    global currentPose
    mStatusLock.acquire()
    if odom != None:
        mStatus.odomStatus = True
        currentPose=odom.pose.pose; #更新坐标

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

def getglobalMoveFlag(moveEn):
    global navthread
    if not moveEn.data:
        #关闭视觉导航
        if not navthread.stopped():
            navthread.stop()

def getNavFlag(navRun):
    global navFlag,navthread
    if navRun.data and not navthread.stopped():
        navFlag=True
    else:
        navFlag=False

def broadcast():
    global reportPub, cmd_pub,mapSave_pub,globalMovePub,elevatorPub,tilt_pub,nav_lastTime
    rospy.init_node("broadcast", anonymous=True)
    nav_lastTime=rospy.Time.now()
    rospy.Subscriber("/xqserial_server/Power", Float64, getPower)
    rospy.Subscriber("/usb_cam/image_raw", Image, getImage)
    rospy.Subscriber("/odom_combined", Odometry, getOdom)
    rospy.Subscriber("/ORB_SLAM/Camera", rospy.msg.AnyMsg, getOrbTrackingFlag)
    rospy.Subscriber("/ORB_SLAM/Frame", Image, getOrbStartStatus)
    rospy.Subscriber("/globalMoveFlag", Bool, getglobalMoveFlag)
    rospy.Subscriber('/nav_setStop', Bool, getNavFlag)
    globalMovePub = rospy.Publisher('/globalMoveFlag', Bool , queue_size=1)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist , queue_size=0)


if __name__ == "__main__":
    broadcast()
    rate = rospy.Rate(10)
    #配置udp广播
    MYPORT = 22001  #广播端口
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    #开启udp接收监听线程
    UserSerThread = UserSer()
    UserSerThread.start()
    i=10
    ii=40
    cmd4="aplay /home/xiaoqiang/Desktop/d.wav"
    while not rospy.is_shutdown():
        # #每２秒提示一下
        # if ii==10:
        #     subprocess.Popen(cmd4,shell=True)

        # if not control_flag and ii>32 and  ii<37:
        #     speed_cmd.linear.x = 0
        #     speed_cmd.angular.z = 0
        #     cmd_pub.publish(speed_cmd)
        #每4秒心跳维护一次
        if ii==40:
            ii=0
            control_flag=False
        ii+=1
        #持续反馈状态
        if UserSocket_remote!=None and UserSerSocket!=None:

            sendData[4:8]=map(ord,struct.pack('f',0.0))
            sendData[8:12]=map(ord,struct.pack('f',0.0))
            sendData[12:16]=map(ord,struct.pack('f',0.0))
            sendData[16:20]=map(ord,struct.pack('f',mStatus.power))
            sendData[24:28]=map(ord,struct.pack('f',0.0))
            if mStatus.odomStatus :
                statu0=0x01 #混合里程计
            else:
                statu0=0x00
            if mStatus.imageStatus :
                statu1=0x02 #视觉摄像头
            else:
                statu1=0x00
            if mStatus.orbStartStatus :
                statu2=0x04 #视觉系统状态
            else:
                statu2=0x00
            if mStatus.orbInitStatus :
                statu3=0x08 #视觉系统状态
            else:
                statu3=0x00
            sendData[20]=statu0+statu1+statu2+statu3
            try:
                UserSerSocket.sendto(bytes(sendData),UserSocket_remote)
            except:
                print "remote disconnect !\n"

        #每秒广播一次
        if i==10:
            i=0;
            data = "xq"
            #发送广播包
            try:
                s.sendto(data, ('<broadcast>', MYPORT))
            except:
                continue
            # clear data
            mStatus.power = 0.0
            mStatus.orbInitStatus = False
            mStatus.orbStartStatus = False
            mStatus.imageStatus = False
            mStatus.odomStatus = False
            # print "getyou4"
            # print str(sendData[20])
        i+=1;
        rate.sleep()
    UserSerThread.stop()
