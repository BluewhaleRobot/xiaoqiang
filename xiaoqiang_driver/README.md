# xiaoqiang driver
xiaoqiang device drivers

## input topic
|name|type|description|
|:--:|:--:|:--|
|cmd_vel|geometry_msgs/Twist|robot velocity|
|infrared_sensor|std_msgs/Bool|infrared sensor enable flag|
|imu_cal|std_msgs/Bool|imu correction flag|

## output topic
|name|type|rate|description|
|:--|:--|:--|:--|
|odom|nav_msgs/Odometry|50hz|robot odometry|
|pose2d|geometry_msgs/Pose2D|50hz|robot pose2d|
|power|std_msgs/Float64|50hz|robot battery voltage|
|infrared_sensor|std_msgs/Int32|50hz|infrared sensor status|
|twist|geometry_msgs/Twist|50hz|robot current twist|
|tf|odom-->base_footprint|50hz|robot tf transform|
|tf_static|base_footprint-->base_link|100hz|robot tf transform|

## input param

|name|default|description|
|:--|:--|:--|
|port|/dev/xiaoqiang|xiaoqiang serial port|
|baud|115200|serial port baud rate|
|wheel_separation|0.37|distance between two wheels|
|wheel_radius|0.06|radius of the wheel|
|debug_flag|false|debug flag|
|max_speed|2.0|max speed of the robot|
|cmd_topic|cmd_vel|topic name to send velocity to|


## Usage:
### download to your ros workspace
```bash
cd [to your workspace]
git clone https://github.com/BlueWhaleRobot/xiaoqiang.git
cd ..
catkin_make
```
### Quickstart
```bash
roslaunch xiaoqiang_driver core.launch
```

## Made with :heart: by BlueWhale Robot.


小强各设备驱动
## 使用方法
#### 安装到小强ROS工作目录
```
cd [到你的工作空间]
git clone https://github.com/BlueWhaleRobot/xiaoqiang.git
cd ..
catkin_make
```
### 直接启动
```
roslaunch xiaoqiang_driver core.launch
```

## 由蓝鲸机器人精 :heart: 制作。
