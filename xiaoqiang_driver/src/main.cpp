/******************************************************************************
*
* The MIT License (MIT)
*
* Copyright (c) 2018 Bluewhale Robot
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Xie fusheng
*         Randoms
*******************************************************************************/

#include <iostream>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include "xiaoqiang_driver/DiffDriverController.h"
#include "xiaoqiang_driver/StatusPublisher.h"
#include "xiaoqiang_driver/AsyncSerial.h"

using namespace std;

int main(int argc, char** argv)
{
  ROS_DEBUG("welcome to xiaoqiang serial server,please feel free at home!");
  ros::init(argc, argv, "xiaoqiang_driver");
  ros::start();

  //获取串口参数
  std::string port;
  ros::param::param<std::string>("~port", port, "/dev/xiaoqiang");
  int baud;
  ros::param::param<int>("~baud", baud, 115200);
  ROS_DEBUG_STREAM("port:" << port << " baud:" << baud);

  //获取小车机械参数
  double separation = 0, radius = 0;
  bool DebugFlag = false;
  ros::param::param<double>("~wheel_separation", separation, 0.37);
  ros::param::param<double>("~wheel_radius", radius, 0.0625);
  ros::param::param<bool>("~debug_flag", DebugFlag, false);
  xiaoqiang_driver::StatusPublisher xq_status(separation, radius);

  //获取小车控制参数
  double max_speed;
  string cmd_topic;
  ros::param::param<double>("~max_speed", max_speed, 2.0);
  ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

  try
  {
    CallbackAsyncSerial serial(port, baud);
    serial.setCallback(boost::bind(&xiaoqiang_driver::StatusPublisher::Update, &xq_status, _1, _2));
    xiaoqiang_driver::DiffDriverController xq_diffdriver(max_speed, cmd_topic, &xq_status, &serial);
    boost::thread cmd2serialThread(&xiaoqiang_driver::DiffDriverController::run, &xq_diffdriver);
    // send test flag
    char debugFlagCmd[] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T' };
    if (DebugFlag)
    {
      ROS_DEBUG_STREAM("Debug mode Enabled");
      serial.write(debugFlagCmd, 5);
    }
    // send reset cmd
    char resetCmd[] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I' };
    serial.write(resetCmd, 5);

    ros::Rate r(50);  //发布周期为50hz
    while (ros::ok())
    {
      if (serial.errorStatus() || serial.isOpen() == false)
      {
        ROS_ERROR_STREAM("Error: serial port closed unexpectedly");
        break;
      }
      xq_status.Refresh();  //定时发布状态
      r.sleep();
      // cout<<"run"<<endl;
    }

  quit:
    serial.close();
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Open " << port << " failed.");
    ROS_ERROR_STREAM("Exception: " << e.what());
  }

  ros::shutdown();
  return 0;
}
