cmake_minimum_required(VERSION 3.28.3)
project(yaml_common)

add_compile_options(-std=c++17 -Wall -Wextra)

option(BUILD_WITH_GEOMETRY_COMMON "Build with geometry_common as dependency" ON)

# The documentation build option set to OFF by default
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOC)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# Optionally find geometry_common
set(dependency "")
if(BUILD_WITH_GEOMETRY_COMMON)
    set(dependency ${dependency} geometry_common)
    add_definitions(-DUSE_GEOMETRY_COMMON)
    find_package(${dependency})
else(BUILD_WITH_GEOMETRY_COMMON)
    message("Building WITHOUT geometry_common dependency")
endif(BUILD_WITH_GEOMETRY_COMMON)

# =========
# LIBRARIES
# =========
set(source_files
    src/Parser2.cpp
)
set(link_libraries
    yaml-cpp::yaml-cpp
)

if(BUILD_WITH_GEOMETRY_COMMON)
    set(source_files ${source_files} src/conversions/GeometryCommon.cpp)
    set(link_libraries ${link_libraries} geometry_common::geometry_common)
endif(BUILD_WITH_GEOMETRY_COMMON)

add_library(yaml_common
    ${source_files}
)
target_include_directories(yaml_common PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(yaml_common PUBLIC
    ${link_libraries}
)

# =======
# INSTALL
# =======
install(
    TARGETS
        yaml_common
    EXPORT
        export_yaml_common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY
        include/
    DESTINATION
        include/${PROJECT_NAME}
)

# =====
# TESTS
# =====
if (BUILD_TESTING)
    add_subdirectory(test)
endif ()


ament_export_dependencies(
    yaml-cpp
    ${dependency}
)

ament_export_targets(export_yaml_common)

ament_package()
