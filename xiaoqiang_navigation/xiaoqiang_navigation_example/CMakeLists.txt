cmake_minimum_required(VERSION 2.8.3)
project(xiaoqiang_navigation_example)

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
  DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/square.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)