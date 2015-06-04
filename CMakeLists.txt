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
      src/vision_util.cpp)

rosbuild_add_executable(depth_detector_node
      src/depth_detector_node.cpp
      src/flow_finder.cpp
      src/sentis_m100.cpp
      src/vision_util.cpp)
include_directories(/usr/include/libm100)
target_link_libraries(depth_detector_node m100)

rosbuild_add_executable(cyl_detector_node
      src/cyl_detector.cpp
      src/cyl_detector_node.cpp)
include_directories(/home/akihiko/install/boost1.49/include)
target_link_libraries(cyl_detector_node vtkCommon vtkFiltering vtkRendering vtkGraphics)

rosbuild_add_executable(flow_analyzer_node
      src/flow_analyzer_node.cpp
      src/sentis_m100.cpp
      src/flow_finder.cpp
      src/flow_analyzer.cpp)
include_directories(/usr/include/libm100)
target_link_libraries(flow_analyzer_node m100)

rosbuild_add_executable(sentis_m100_node
      src/sentis_m100_node.cpp
      src/sentis_m100.cpp)
include_directories(/usr/include/libm100)
target_link_libraries(sentis_m100_node m100)

rosbuild_add_executable(ode_pour_sim_node
      src/ode_pour_sim_node.cpp
      src/ode_pour_sim.cpp)
add_definitions(-DdDOUBLE)
include_directories(/home/akihiko/prg/libode/ode-latest/include)
target_link_libraries(ode_pour_sim_node
      /home/akihiko/prg/libode/ode-latest/ode/src/.libs/libode.a
      /home/akihiko/prg/libode/ode-latest/drawstuff/src/.libs/libdrawstuff.a
      SM ICE GL Xext X11 GLU)

rosbuild_add_executable(ode_grpour_sim_node
      src/ode_grpour_sim_node.cpp
      src/ode_grpour_sim.cpp)
add_definitions(-DdDOUBLE)
include_directories(/home/akihiko/prg/libode/ode-latest/include)
target_link_libraries(ode_grpour_sim_node
      /home/akihiko/prg/libode/ode-latest/ode/src/.libs/libode.a
      /home/akihiko/prg/libode/ode-latest/drawstuff/src/.libs/libdrawstuff.a
      SM ICE GL Xext X11 GLU)

# -L/usr/X11R6/lib -ldl  -lpthread


file(GLOB raytrace_srcs1 "include/pr2_lfd_vision/raytrace/doncross/*.cpp")
file(GLOB raytrace_srcs2 "include/pr2_lfd_vision/raytrace/lodepng/*.cpp")
rosbuild_add_library(ray_trace ${raytrace_srcs1} ${raytrace_srcs2})

rosbuild_add_executable(rt_pose_estimator_node
      src/rt_pose_estimator_node.cpp
      src/raytrace/pose_estimator.cpp
      src/raytrace/depthscene.cpp
      src/pcl_util.cpp)
target_link_libraries(rt_pose_estimator_node ray_trace)

