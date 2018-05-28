#!/usr/bin/env python
# coding: UTF-8
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
# Author: Randoms

import fcntl
import os
import signal
import sys
import termios
import thread

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def signal_handler(signal, frame):
    os.system('reset')
    rospy.loginfo("Bye.")
    rospy.signal_shutdown("exit")



CURRENT_KEY = ""
CURRENT_ODOM = Odometry()
KEY_CACHE = []
UP_KEY = [27, 91, 65]
DOWN_KEY = [27, 91, 66]
LEFT_KEY = [27, 91, 68]
RIGHT_KEY = [27, 91, 67]


def update_speed(odom):
    global CURRENT_ODOM
    CURRENT_ODOM = odom

def send_cmd(key):
    cmd = Twist()
    cmd.linear.x = CURRENT_ODOM.twist.twist.linear.x
    cmd.angular.z = CURRENT_ODOM.twist.twist.angular.z
    cmd.linear.x = 0
    cmd.angular.z = 0
    if key == "up":
        cmd.linear.x = 0.2
    elif key == "down":
        cmd.linear.x = -0.2
    elif key == "left":
        cmd.angular.z = 0.8
    elif key == "right":
        cmd.angular.z = -0.8
    cmd_pub.publish(cmd)


def get_current_key():
    global CURRENT_KEY, KEY_CACHE
    while True:
        try:
            c = sys.stdin.read(1)
            if ord(c) == 27:
                # somestrange pressed
                KEY_CACHE = [27]
                continue
            elif len(KEY_CACHE) != 0:
                KEY_CACHE.append(ord(c))
                if KEY_CACHE == LEFT_KEY:
                    CURRENT_KEY = "left"
                if KEY_CACHE == RIGHT_KEY:
                    CURRENT_KEY = "right"
                if KEY_CACHE == UP_KEY:
                    CURRENT_KEY = "up"
                if KEY_CACHE == DOWN_KEY:
                    CURRENT_KEY = "down"
                if len(KEY_CACHE) == 3:
                    KEY_CACHE = []
            else:
                CURRENT_KEY = c
        except:
            pass



if __name__ == "__main__":

    rospy.init_node('xiaoqiang_controller', anonymous=True)
    rospy.loginfo("Welcome to use xiaoqiang controller.")
    rospy.loginfo("Use arrow keys to move robot around.")
    rospy.loginfo("Use space to stop robot.")
    rospy.loginfo("Use Ctrl + C to exit.")

    # init terminal
    signal.signal(signal.SIGINT, signal_handler)

    fd = sys.stdin.fileno()

    old_term = termios.tcgetattr(fd)
    new_attr = termios.tcgetattr(fd)
    new_attr[3] = new_attr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, new_attr)

    old_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
    rospy.Subscriber("/xiaoqiang_driver/odom", Odometry, update_speed)

    thread.start_new_thread(get_current_key, ())

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        send_cmd(CURRENT_KEY)
        rate.sleep()
