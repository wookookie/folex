cmake_minimum_required(VERSION 3.0.2)
project(folex_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
  DEPENDS
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/folex_controller
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/folex_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############
