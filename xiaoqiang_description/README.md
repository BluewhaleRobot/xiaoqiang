# xiaoqiang_urdf

xiaoqiang urdf model files

## input topic

none

## output topic
|name|type|
|:--|:--|
|/joint_states|sensor_msgs/JointState|

## published tf transformation

|name|type|
|:--|:--|
|/tf|base_link-->right_wheel base_link-->left_wheel|
|/tf|base_link-->back_wheel|

## Usage:

### download to xiaoqiang ros workspace

```bash
cd [to your workspace]
git clone https://github.com/BlueWhaleRobot/xiaoqiang.git 
cd ..
catkin_make
```

### Quickstart    

```bash
roslaunch xiaoqiang_description xiaoqiang_model.launch
```
## Made with :heart: by BlueWhale Tech corp.


小强的模型相关文件

## 使用方法：
### 安装到小强ROS工作目录

```bash
cd [到你的工作空间]
git clone https://github.com/BlueWhaleRobot/xiaoqiang.git
cd ..
catkin_make
```

### 直接启动

```bash
roslaunch xiaoqiang_description xiaoqiang_model.launch
```

## 由蓝鲸机器人精 :heart: 制作。
