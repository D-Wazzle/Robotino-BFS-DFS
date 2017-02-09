cmake_minimum_required(VERSION 2.8.3)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(robotino_control_bfdfnav)

set(
  CMAKE_MODULE_PATH
  /usr/local/robotino/api2/cmake
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

find_package(catkin REQUIRED COMPONENTS
   cv_bridge
   image_transport
   robotino_msgs
   roscpp
   roscpp_serialization
   rosconsole
   rospy
   std_msgs
   robotino_node
   robotino_description
   OpenCV
)

find_package(RobotinoAPI2)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS robotino_node robotino_description robotino_msgs roscpp rospy std_msgs roscpp_serialization rosconsole
   DEPENDS opencv robotino_node robotino_description robotino_msgs roscpp rospy std_msgs roscpp_serialization rosconsole)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_LIBRARY_DIRS} ${OpenCV_INCLUDE_DIRS})

#rosbuild_add_executable(talker src/talker.cpp)
#rosbuild_add_executable(listener src/listener.cpp)

add_executable(BHA src/BHA.cpp)
target_link_libraries(BHA ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})

add_executable(object_grab src/BHA.cpp)
target_link_libraries(object_grab ${REC_ROBOTINO_API2_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(vision src/vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${REC_ROBOTINO_API2_LIBRARY})

add_executable(analyzeColor src/analyzeColor.cpp)
target_link_libraries(analyzeColor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(poser src/poser.cpp)
target_link_libraries(poser ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})

add_executable(BHApose src/BHApose.cpp)
target_link_libraries(BHApose ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})

add_executable(
  approach_grab_node
  src/approach_grab_node.cpp
  src/approach_grab.cpp)
target_link_libraries(approach_grab_node ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})

add_executable(
  PFNavNode
  src/PFNavNode.cpp
  src/potentialField.cpp
  src/PFNav.cpp
  src/robotinoNavigator.cpp
  src/approach_grab.cpp)
target_link_libraries(PFNavNode ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})
  
add_executable(
  BFNavigatorNode
  src/BFNavigatorNode.cpp
  src/BFNavigator.cpp
  src/map.cpp
  src/searchTreeNode.cpp
  src/breadthFirst.cpp
  src/robotinoNavigator.cpp
  src/approach_grab.cpp)
target_link_libraries(BFNavigatorNode ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})

add_executable(
  DFNavigatorNode
  src/DFNavigatorNode.cpp
  src/DFNavigator.cpp
  src/map.cpp
  src/searchTreeNode.cpp
  src/depthFirst.cpp
  src/robotinoNavigator.cpp
  src/approach_grab.cpp)
target_link_libraries(DFNavigatorNode ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})

# openCV 

#find_package(OpenCV)
#include_directories(${OpenCV_INCLUDE_DIRS})

