cmake_minimum_required(VERSION 3.5.1)
project(yaml_common)

add_compile_options(-std=c++11 -Wall -Wextra)

option(BUILD_WITH_GEOMETRY_COMMON "Build with geometry_common as dependency" ON)

if (WIN32)
    find_package(yaml-cpp REQUIRED)
else (WIN32)
    set(YAML_CPP_LIBRARIES yaml-cpp)
endif (WIN32)

set(dependency "")
if(BUILD_WITH_GEOMETRY_COMMON)
    set(dependency ${dependency} geometry_common)
    message("Building with geometry_common dependency")
    add_definitions(-DUSE_GEOMETRY_COMMON)
endif(BUILD_WITH_GEOMETRY_COMMON)

find_package(catkin REQUIRED COMPONENTS
    ${dependency}
)

find_package (yaml-cpp REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        ${dependency}
    INCLUDE_DIRS
        include
        ${YAML_CPP_INCLUDE_DIRS}
    LIBRARIES
        yaml_common
        ${YAML_CPP_LIBRARIES}
)

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

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# =========
# LIBRARIES
# =========
set(source_files
    src/Parser.cpp
    src/Parser2.cpp
)

if(BUILD_WITH_GEOMETRY_COMMON)
    set(source_files ${source_files} src/conversions/GeometryCommon.cpp)
endif(BUILD_WITH_GEOMETRY_COMMON)

add_library(yaml_common
    ${source_files}
)
target_link_libraries(yaml_common
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

# =====
# TESTS
# =====
if (CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif ()


# =======
# INSTALL
# =======
install(TARGETS yaml_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
