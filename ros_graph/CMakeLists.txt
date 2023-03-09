project(ros_graph)

set(ROS_GRAPH_LIBRARY_NAME ${PROJECT_NAME}_static)

include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(${ROS_GRAPH_LIBRARY_NAME} STATIC
    src/worker/depend_graph.cpp
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
