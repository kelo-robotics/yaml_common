find_package(ament_cmake_gtest REQUIRED)

# NOTE: all test must end with "_test.cpp" !
FILE(GLOB TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*_test.cpp" )

ament_add_gtest(yaml_common_test
    main.cpp
    ${TEST_SOURCES}
)
target_link_libraries(yaml_common_test
    yaml_common
)
