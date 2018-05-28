cmake_minimum_required(VERSION 2.8.3)
project(xiaoqiang_monitor)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   Status.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
