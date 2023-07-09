cmake_minimum_required(VERSION 3.20)
project(ros_tools)

set(WORKSPACE_ROOT_DIR ${PROJECT_SOURCE_DIR})

option(UNITTEST "do unittest or not" OFF)

add_subdirectory(ros_common)
add_subdirectory(ros_graph)
