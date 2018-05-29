xiaoqiang_freenect_stack
==============

libfreenect based ROS driver
Modified version of xiaoqiang_freenect_stack, add tilt angle control support


## tilt motor control usage：

#### 1.launch the driver
```
roslaunch xiaoqiang_freenect freenect-xyz.launch
```
#### 2.publish tilt motor position topic
```
rostopic pub set_tilt_degree std_msgs/Int16 '{data: -20}' -r 1
```
{data: -20} is the motor position,you can change -20 to any integer number in [-30 30].

## Made with ❤️ by Bluewhale Robot.
