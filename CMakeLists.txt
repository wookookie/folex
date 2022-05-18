cmake_minimum_required(VERSION 3.15)
project(folex CXX)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(src/actuator)
add_subdirectory(src/folex)
add_subdirectory(src/trajectory)
