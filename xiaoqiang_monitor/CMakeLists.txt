cmake_minimum_required(VERSION 2.8.3)
project(xiaoqiang_monitor)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rospy
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  src/monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)