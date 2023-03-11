project(ros_graph)

set(ROS_GRAPH_LIBRARY_NAME ${PROJECT_NAME}_static)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${ROS_GRAPH_LIBRARY_NAME} STATIC
    src/depend_graph.cpp
    src/worker/worker.cpp
)
target_link_libraries(${ROS_GRAPH_LIBRARY_NAME}
    fmt
    gflags
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${ROS_GRAPH_LIBRARY_NAME}
)

add_executable(unittest
    test/worker/graph_test.cpp
)
target_link_libraries(unittest
    ${ROS_GRAPH_LIBRARY_NAME}
    gtest_main
    gtest
)
