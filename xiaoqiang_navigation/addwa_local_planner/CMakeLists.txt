cmake_minimum_required(VERSION 2.8.3)
project(addwa_local_planner)
find_package(catkin REQUIRED)
catkin_package()

install(
  FILES  lib/libaddwa_local_planner.so
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)