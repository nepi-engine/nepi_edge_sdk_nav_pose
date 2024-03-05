#[[
NEPI Dual-Use License
Project: nepi_edge_sdk_nav_pose

This license applies to any user of NEPI Engine software

Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
see https://github.com/numurus-nepi/nepi_edge_sdk_nav_pose

This software is dual-licensed under the terms of either a NEPI software developer license
or a NEPI software commercial license.

The terms of both the NEPI software developer and commercial licenses
can be found at: www.numurus.com/licensing-nepi-engine

Redistributions in source code must retain this top-level comment block.
Plagiarizing this software to sidestep the license obligations is illegal.

Contact Information:
====================
- https://www.numurus.com/licensing-nepi-engine
- mailto:nepi@numurus.com

]]
cmake_minimum_required(VERSION 2.8.3)
project(nepi_edge_sdk_nav_pose)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wall)

# Need the hostname because OpenCV_DIR location depends on which host -- num-tx2 uses Jetpack-installed opencv4
cmake_host_system_information( RESULT BUILDHOST QUERY HOSTNAME )
if( ${BUILDHOST} STREQUAL "num-tx2" )
    set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)
else()
    set(OpenCV_DIR /usr/share/OpenCV)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nepi_edge_sdk_base nepi_ros_interfaces cv_bridge OpenCV pcl_ros pcl_conversions tf2_ros)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Use the verbose version of make
set(CMAKE_VERBOSE_MAKEFILE ON)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#    FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs  # Or other packages containing msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS nepi_edge_sdk_base nepi_ros_interfaces

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Nav/Pos Mgr Node
add_executable(nav_pose_mgr_node
  src/nav_pose_mgr.cpp
)

add_dependencies(nav_pose_mgr_node
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(nav_pose_mgr_node
  ${catkin_LIBRARIES})

## GPSD ROS Client Node
add_executable(gpsd_ros_client_node
  src/gpsd_ros_client.cpp)

add_dependencies(gpsd_ros_client_node
  ${catkin_EXPORTED_TARGETS}
  nepi_gpsd)

target_include_directories(gpsd_ros_client_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/nepi_gpsd/include
)

target_link_libraries(gpsd_ros_client_node
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/nepi_gpsd/gpsd-3.22.1~dev/libgps.so
)

# Rule to build gpsd, including client library libgps.so
#add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/nepi_gpsd/gpsd-3.22.1~dev/libgps.so
#  COMMAND scons
#  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/nepi_gpsd
#  COMMENT "Building special Numurus version of GPSD and libgps.so"
#)
add_custom_target(nepi_gpsd ALL
  COMMAND scons # Build command for full gpsd project
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/nepi_gpsd
  COMMENT "Building special Numurus version of GPSD and libgps.so"
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#  scripts/time_sync_mgr.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  nav_pose_mgr_node
  gpsd_ros_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation - Msgs and Services (generated headers) are under the project name subdir
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Non-generated headers are located directly in include and get copied to the root of include in the install directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# GPSD Client Library
install(FILES
  nepi_gpsd/gpsd-3.22.1~dev/libgps.so
  nepi_gpsd/gpsd-3.22.1~dev/libgps.so.28
  nepi_gpsd/gpsd-3.22.1~dev/libgps.so.28.0.0
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# GPSD Server Application
install(PROGRAMS
  nepi_gpsd/gpsd-3.22.1~dev/gpsd/gpsd
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install etc files (startup scripts, factory configs, etc.) to the global (not project) etc destination
install(DIRECTORY etc/
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
