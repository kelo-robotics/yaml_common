# NOTE: all test must end with "_test.cpp" !
FILE(GLOB TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*_test.cpp" )

catkin_add_gtest(yaml_common_test
    main.cpp
    ${TEST_SOURCES}
)
target_link_libraries(yaml_common_test
    ${catkin_LIBRARIES}
    yaml_common
)
