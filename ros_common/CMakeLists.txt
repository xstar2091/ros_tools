project(ros_common)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCE_DIR}/src/cmdkey/cmdkey.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
