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

#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "xiaoqiang_driver/StatusPublisher.h"
#include "xiaoqiang_driver/AsyncSerial.h"

namespace xiaoqiang_driver
{
class DiffDriverController
{
public:
  DiffDriverController();
  DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher* xq_status_,
                       CallbackAsyncSerial* cmd_serial_);
  void run();
  void sendcmd(const geometry_msgs::Twist& command);
  void imuCalibration(const std_msgs::Bool& calFlag);
  void setStatusPtr(StatusPublisher& status);
  void updateMoveFlag(const std_msgs::Bool& moveFlag);
  void updateBarDetectFlag(const std_msgs::Bool& DetectFlag);

private:
  double max_wheelspeed;  //单位为转每秒,只能为正数
  std::string cmd_topic;
  StatusPublisher* xq_status;
  CallbackAsyncSerial* cmd_serial;
  boost::mutex mMutex;
  bool MoveFlag;
};
}
#endif  // DIFFDRIVERCONTROLLER_H
