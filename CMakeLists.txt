cmake_minimum_required(VERSION 3.15)
project(folex)

set(CMAKE_CXX_STANDARD 14)

add_executable(folex
  src/actuator.cpp
  src/folex.cpp
)

target_link_libraries(folex
  pthread
  # x86-64
  /usr/local/lib/libdxl_x64_cpp.so
  # ARM
  #/usr/local/lib/libdxl_sbc_cpp.so
)

target_include_directories(folex
PRIVATE include/folex
        /usr/local/include/dynamxel_sdk
)
