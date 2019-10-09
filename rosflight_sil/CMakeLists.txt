cmake_minimum_required(VERSION 2.8.3)
project(rosflight_sil)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Default to release build for speed
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  rosflight_sim
  rosflight_firmware
  rosflight_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if (NOT TARGET utils)
  include_directories(lib/utils/include)
  add_subdirectory(lib/utils)
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs rosflight_sim rosflight_firmware rosflight_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosflight_sil_node
  src/rosflight_sil.cpp
  src/sil_board.cpp
  src/rosflight_sil_node.cpp
  src/fixedwing_forces_and_moments.cpp
  src/multirotor_forces_and_moments.cpp
)
target_link_libraries(rosflight_sil_node utils ${catkin_LIBRARIES})

