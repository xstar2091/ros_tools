project(ros_graph)

set(ROS_GRAPH_LIBRARY_NAME ${PROJECT_NAME}_static)

find_package(Boost REQUIRED COMPONENTS
    filesystem
    system
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_library(${ROS_GRAPH_LIBRARY_NAME} STATIC
    src/command_line_param.cpp
    src/depend_graph.cpp
    src/worker/worker.cpp
    src/worker/depend/depend_worker_param.cpp
    src/worker/depend/depend_worker.cpp
    src/worker/dependby/dependby_worker_param.cpp
    src/worker/dependby/dependby_worker.cpp
    src/worker/dependtree/dependtree_worker_param.cpp
    src/worker/dependtree/dependtree_worker.cpp
    src/worker/dependtreeby/dependtreeby_worker_param.cpp
    src/worker/dependtreeby/dependtreeby_worker.cpp
)
target_link_directories(${ROS_GRAPH_LIBRARY_NAME} PRIVATE
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${ROS_GRAPH_LIBRARY_NAME}
    ros_common
    ${Boost_LIBRARIES}
    fmt
    gflags
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${ROS_GRAPH_LIBRARY_NAME}
)

add_executable(unittest
    test/command_line_param_test.cpp
    test/graph_test.cpp
)
target_link_libraries(unittest
    ${ROS_GRAPH_LIBRARY_NAME}
    ${Boost_LIBRARIES}
    gtest_main
    gtest
)
