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

#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <algorithm>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"

#define PI 3.14159265

namespace xiaoqiang_driver
{
typedef struct
{
  int status;               //小车状态，0表示未初始化，1表示正常，-1表示error
  float power;              //电源电压【9 13】v
  float theta;              //方位角，【0 360）°
  int encoder_ppr;          //车轮1转对应的编码器个数
  int encoder_delta_r;      //右轮编码器增量， 个为单位
  int encoder_delta_l;      //左轮编码器增量， 个为单位
  int encoder_delta_car;    //两车轮中心位移，个为单位
  int omga_r;               //右轮转速 个每秒
  int omga_l;               //左轮转速 个每秒
  float distance1;          //第一个超声模块距离值 单位cm
  float distance2;          //第二个超声模块距离值 单位cm
  float distance3;          //第三个超声模块距离值 单位cm
  float distance4;          //第四个超声模块距离值 单位cm
  float IMU[9];             // mpu9250 9轴数据
  unsigned int time_stamp;  //时间戳
} UPLOAD_STATUS;

class StatusPublisher
{
public:
  StatusPublisher();
  StatusPublisher(double separation, double radius);
  void Refresh();
  void Update(const char* data, unsigned int len);
  double get_wheel_separation();
  double get_wheel_radius();
  int get_wheel_ppr();
  int get_status();
  geometry_msgs::Pose2D get_CarPos2D();
  void get_wheel_speed(double speed[2]);
  geometry_msgs::Twist get_CarTwist();
  std_msgs::Float64 get_power();
  nav_msgs::Odometry get_odom();
  UPLOAD_STATUS car_status;

private:
  // Wheel separation, wrt the midpoint of the wheel width: meters
  double wheel_separation;

  // Wheel radius (assuming it's the same for the left and right wheels):meters
  double wheel_radius;

  geometry_msgs::Pose2D CarPos2D;  //小车开始启动原点坐标系
  geometry_msgs::Twist CarTwist;   //小车自身坐标系
  std_msgs::Float64 CarPower;      // 小车电池信息
  nav_msgs::Odometry CarOdom;      // 小车位置和速度信息
  ros::NodeHandle mNH;
  ros::Publisher mPose2DPub;
  ros::Publisher mTwistPub;
  ros::Publisher mStatusFlagPub;
  ros::Publisher mPowerPub;
  ros::Publisher mOdomPub;
  ros::Publisher pub_barpoint_cloud_;
  ros::Publisher pub_clearpoint_cloud_;

  bool mbUpdated;

  boost::mutex mMutex;
};

}  // namespace xiaoqiang_driver

#endif  // STATUSPUBLISHER_H
