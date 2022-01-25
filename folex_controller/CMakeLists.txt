cmake_minimum_required(VERSION 3.0.2)
project(folex_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
)

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    # Message
    sensor_msgs
    std_msgs
  DEPENDS
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/folex_controller
)

## Declare a C++ executable
add_executable(folex_controller src/folex_controller.cpp)
target_link_libraries(folex_controller ${catkin_LIBRARIES})
