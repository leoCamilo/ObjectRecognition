cmake_minimum_required(VERSION 3.0.2)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# ---------------- H E R E   I S   M Y   C O D E ---------------- #

# include_directories(include ${PROJECT_SOURCE_DIR}/include)

set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_PREFIX_PATH $ENV{HOME}/Qt5.6.0/5.6/gcc_64)

find_package (Qt5Widgets)
find_package (Threads)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

rosbuild_add_library (mainwindow src/main/main_screen.cpp)
target_link_libraries (mainwindow Qt5::Widgets)

rosbuild_add_library (ros_thread src/main/ros-thread.cpp)
rosbuild_add_library (gui_thread src/main/gui-thread.cpp)
rosbuild_add_library (cloud_lib  lib/cloud-tools.cpp)

target_link_libraries (gui_thread mainwindow)
target_link_libraries (ros_thread cloud_lib)

# compile main source
rosbuild_add_executable (kinect_config src/main/main.cpp)

# add dependencies on main bin
target_link_libraries (kinect_config ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (kinect_config mainwindow ros_thread gui_thread)

rosbuild_add_executable (main src/nodes/main.cpp)
rosbuild_add_executable (ransac src/nodes/ransac.cpp)
rosbuild_add_executable (viewer src/nodes/pcl-viewer.cpp)
rosbuild_add_executable (vfh src/nodes/vfh-classifier.cpp)
rosbuild_add_executable (classify src/nodes/classifier.cpp)
rosbuild_add_executable (vfh-tracker src/nodes/vfh-tracking.cpp)
rosbuild_add_executable (snapshot src/nodes/take-snapshot.cpp)
rosbuild_add_executable (tracking src/nodes/prediction-tracking.cpp)
rosbuild_add_executable (downsample src/nodes/voxel-downsample.cpp)
rosbuild_add_executable (clustering src/nodes/euclidian-clustering.cpp)
rosbuild_add_executable (association src/nodes/knn-association.cpp)

target_link_libraries (vfh cloud_lib)
target_link_libraries (main cloud_lib)
target_link_libraries (ransac cloud_lib)
target_link_libraries (classify cloud_lib)
target_link_libraries (snapshot cloud_lib)
target_link_libraries (tracking cloud_lib)
target_link_libraries (vfh-tracker cloud_lib)
target_link_libraries (downsample cloud_lib)
target_link_libraries (clustering cloud_lib)
target_link_libraries (association cloud_lib)
