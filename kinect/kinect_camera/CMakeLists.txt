cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include ($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops)
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

pkg_check_modules (LIBUSB REQUIRED libusb-1.0)
include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})

# ---[ Kinect Driver
rosbuild_add_library (kinect_driver src/kinect_driver.cpp)
rosbuild_add_compile_flags (kinect_driver ${SSE_FLAGS})

rosbuild_add_library (kinect_driver_nodelets src/kinect_driver_nodelets.cpp)
rosbuild_add_compile_flags (kinect_driver_nodelets ${SSE_FLAGS})
target_link_libraries (kinect_driver_nodelets kinect_driver)

rosbuild_add_executable (kinect_node src/kinect_node.cpp)
target_link_libraries (kinect_node kinect_driver)
