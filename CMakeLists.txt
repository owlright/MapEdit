cmake_minimum_required(VERSION 3.10)
project(SeparateFrames)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(separate_frames separate_frames.cpp)
target_link_libraries(separate_frames ${PCL_LIBRARIES})
