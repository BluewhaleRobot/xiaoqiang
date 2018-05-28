# startup

xiaoqiang launch file package, including launch files to start many devices

## Usage:
### download to xiaoqiang ros workspace
```
cd [to your workspace]
git clone https://github.com/BluewhaleRobot/xiaoqiang.git
cd ..
catkin_make
```   
### Install this service into system, service name is startup
```
rosrun robot_upstart install xiaoqiang_bringup/launch/xiaoqiang_robot.launch
```
### Uninstall this service
```
rosrun robot_upstart uninstall xiaoqiang_robot
```
### start this service manually
```
sudo service xiaoqiang_robot start
```
### stop this service
```
sudo service xiaoqiang_robot stop
```
## Made with :heart: by Bluewhale Robot.


小强启动文件包，包含了小强启动各种硬件的launch文件

## 使用方法:
### 首先将软件包下载到小强ROS工作目录
```
cd [到你的工作空间]
git clone https://github.com/BlueWhaleRobot/xiaoqiang.git
cd ..
catkin_make
```   
### 将软件包中的开机启动服务注册到系统
```
rosrun robot_upstart install xiaoqiang_bringup/launch/xiaoqiang_robot.launch
```
### 移除startup服务
```
rosrun robot_upstart uninstall xiaoqiang_robot
```
### 开启startup服务
```
sudo service xiaoqiang_robot start
```
### 关闭startup服务
```
sudo service xiaoqiang_robot stop
```
## 由蓝鲸机器人精 :heart: 制作。
