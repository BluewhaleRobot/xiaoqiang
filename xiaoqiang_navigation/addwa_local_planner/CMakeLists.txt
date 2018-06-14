cmake_minimum_required(VERSION 2.8.3)
project(addwa_local_planner)
find_package(catkin REQUIRED)
catkin_package()

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

# check architecture before install
#if( ${ARCHITECTURE} STREQUAL "x86_64" )
#  install(
#    FILES  lib/libaddwa_local_planner.so
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()