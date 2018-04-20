cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

ROSBUILD_INIT()

ROSBUILD_ADD_BOOST_DIRECTORIES()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(depthimg_to_laserscan_converter src/depth_to_laser.cpp)
TARGET_LINK_LIBRARIES(depthimg_to_laserscan_converter ${libs})

ROSBUILD_ADD_EXECUTABLE(get_transform src/get_transform.cpp)
TARGET_LINK_LIBRARIES(get_transform ${libs})
ROSBUILD_LINK_BOOST(get_transform system)
