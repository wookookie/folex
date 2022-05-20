cmake_minimum_required(VERSION 3.15)

add_library(actuator STATIC
  actuator.cpp
)

target_include_directories(actuator
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    /usr/local/include
)

target_link_libraries(actuator
  PRIVATE
    /usr/local/lib/libdxl_sbc_cpp.so
)
