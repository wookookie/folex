cmake_minimum_required(VERSION 3.15)
project(socket)

add_library(socket
  socket_client.cpp
)

target_include_directories(socket
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
