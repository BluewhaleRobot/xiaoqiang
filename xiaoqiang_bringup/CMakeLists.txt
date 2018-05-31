cmake_minimum_required(VERSION 2.8.3)
project(xiaoqiang_bringup)


find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rospy
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY camera_info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
