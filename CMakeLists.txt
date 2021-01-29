cmake_minimum_required(VERSION 3.0.2)
project(groundtruth_odom)

###########
## Build ##
###########

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

#############
## Package ##
#############

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)
find_package (Eigen3 3.3 REQUIRED)

############
## Catkin ##
############

catkin_package(
  DEPENDS PCL 

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

################
## Executable ##
################

add_library (GroundtruthProvider src/groundtruthprovider.cpp src/groundtruthprovider.h)

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} Eigen3::Eigen GroundtruthProvider)
