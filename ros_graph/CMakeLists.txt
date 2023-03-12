project(ros_graph)

set(ROS_GRAPH_LIBRARY_NAME ${PROJECT_NAME}_static)

find_package(Boost REQUIRED COMPONENTS
    filesystem
    system
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)

add_library(${ROS_GRAPH_LIBRARY_NAME} STATIC
    src/command_line_param.cpp
    src/depend_graph.cpp
    src/worker/worker.cpp
    src/worker/depend/depend_worker.cpp
)
target_link_libraries(${ROS_GRAPH_LIBRARY_NAME}
    ${Boost_LIBRARIES}
    fmt
    gflags
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${ROS_GRAPH_LIBRARY_NAME}
)

add_executable(unittest
    test/graph_test.cpp
)
target_link_libraries(unittest
    ${ROS_GRAPH_LIBRARY_NAME}
    gtest_main
    gtest
)
