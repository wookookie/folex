cmake_minimum_required(VERSION 3.15)

add_executable(folex
  folex.cpp
  main.cpp
  quadruped_task.cpp
)

target_include_directories(folex
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/actuator
    ${CMAKE_SOURCE_DIR}/src/trajectory
)

target_link_libraries(folex
  PRIVATE
    actuator
    pthread
    trajectory
)
