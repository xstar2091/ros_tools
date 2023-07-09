project(ros_common)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCE_DIR}/src/cmdkey/cmdkey_info.cpp
    ${PROJECT_SOURCE_DIR}/src/cmdkey/cmdkey_table.cpp
    ${PROJECT_SOURCE_DIR}/src/cmdkey/cmdkey.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

if (UNITTEST)
    set(UNITTEST_BIN_NAME "unittest_${PROJECT_NAME}")
    add_executable(${UNITTEST_BIN_NAME}
        test/cmdkey/cmdkey_info_test.cpp
        test/cmdkey/cmdkey_test.cpp
    )
    target_include_directories(${UNITTEST_BIN_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )
    target_link_libraries(${UNITTEST_BIN_NAME} PRIVATE
        ${PROJECT_NAME}
        gtest_main
        gtest
    )
endif ()
