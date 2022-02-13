cmake_minimum_required(VERSION 3.0.2)
project(folex_controller)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # Dynamixel SDK
  dynamixel_sdk
  # Message
  sensor_msgs
  std_msgs
)

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    # Dynamixel SDK
    dynamixel_sdk
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

add_executable(actuator src/actuator.cpp)
target_link_libraries(actuator ${catkin_LIBRARIES})
