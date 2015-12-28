cmake_minimum_required(VERSION 2.4.6)
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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(color_detector_node
      src/color_detector_node.cpp
      src/color_detector.cpp
      src/flow_finder.cpp
      src/mov_detector.cpp
      src/vision_util.cpp)
target_link_libraries(color_detector_node ${OpenCV_LIBRARIES})

# rosbuild_add_executable(test_col_flow
#       test/test_col_flow.cpp
#       src/color_detector.cpp
#       src/flow_finder.cpp
#       src/vision_util.cpp)

if(EXISTS /usr/include/libm100)
  rosbuild_add_executable(depth_detector_node
        src/depth_detector_node.cpp
        src/flow_finder.cpp
        src/sentis_m100.cpp
        src/vision_util.cpp)
  include_directories(/usr/include/libm100)
  target_link_libraries(depth_detector_node ${OpenCV_LIBRARIES} m100)
endif(EXISTS /usr/include/libm100)
  
rosbuild_add_executable(cyl_detector_node
      src/cyl_detector.cpp
      src/cyl_detector_node.cpp)
if(EXISTS $ENV{HOME}/install/boost1.49/include)
  include_directories($ENV{HOME}/install/boost1.49/include)
endif(EXISTS $ENV{HOME}/install/boost1.49/include)
target_link_libraries(cyl_detector_node ${OpenCV_LIBRARIES} vtkCommon vtkFiltering vtkRendering vtkGraphics)

if(EXISTS /usr/include/libm100)
  rosbuild_add_executable(flow_analyzer_node
        src/flow_analyzer_node.cpp
        src/sentis_m100.cpp
        src/flow_finder.cpp
        src/flow_analyzer.cpp)
  include_directories(/usr/include/libm100)
  target_link_libraries(flow_analyzer_node ${OpenCV_LIBRARIES} m100)
endif(EXISTS /usr/include/libm100)

if(EXISTS /usr/include/libm100)
  rosbuild_add_executable(sentis_m100_node
        src/sentis_m100_node.cpp
        src/sentis_m100.cpp)
  include_directories(/usr/include/libm100)
  target_link_libraries(sentis_m100_node ${OpenCV_LIBRARIES} m100)
endif(EXISTS /usr/include/libm100)



file(GLOB raytrace_srcs1 "include/lfd_vision/raytrace/doncross/*.cpp")
file(GLOB raytrace_srcs2 "include/lfd_vision/raytrace/lodepng/*.cpp")
rosbuild_add_library(ray_trace ${raytrace_srcs1} ${raytrace_srcs2})

rosbuild_add_executable(rt_pose_estimator_node
      src/rt_pose_estimator_node.cpp
      src/raytrace/pose_estimator.cpp
      src/raytrace/depthscene.cpp
      src/pcl_util.cpp
      src/vision_util.cpp)
target_link_libraries(rt_pose_estimator_node ray_trace ${OpenCV_LIBRARIES})

