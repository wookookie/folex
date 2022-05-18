cmake_minimum_required(VERSION 3.15)

add_library(trajectory STATIC
  trajectory.cpp
)

target_include_directories(trajectory
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
