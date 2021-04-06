cmake_minimum_required(VERSION 3.5)
project(habitrack-superbuild)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/external)


set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)
message(STATUS "${PROJECT_NAME} staged install: ${STAGED_INSTALL_PREFIX}")

include(external/eigen3)
include(external/spdlog)
include(external/mild)
include(external/ceres)
include(external/cereal)
include(external/cxxopts)
include(external/opencv)
include(external/qt)

option(BUILD_DEPS_ONLY "Build only dependencies and nothing else (used for development)" FALSE)

set(habitrack_dependencies
    eigen3_external
    spdlog_external
    mild_external
    ceres_external
    cereal_external
    cxxopts_external
    opencv_external
    qt5base_external
    qt5multimedia_external
)

if (${BUILD_DEPS_ONLY})
    add_library(dummy INTERFACE)
    add_dependencies(dummy ${habitrack_dependencies})
    configure_file(cmake_habitrack.in cmake_habitrack)
else ()
    include(ExternalProject)
    ExternalProject_Add(habitrack
        DEPENDS
            ${habitrack_dependencies}
        SOURCE_DIR
            ${PROJECT_SOURCE_DIR}/..
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DEigen3_DIR=${Eigen3_DIR}
            -Dspdlog_DIR=${spdlog_DIR}
            -Dmild_DIR=${mild_DIR}
            -DCeres_DIR=${Ceres_DIR}
            -Dcereal_DIR=${cereal_DIR}
            -Dcxxopts_DIR=${cxxopts_DIR}
            -DOpenCV_DIR=${OpenCV_DIR}
            -DQt5Concurrent_DIR=${Qt5Concurrent_DIR}
            -DQt5Widgets_DIR=${Qt5Widgets_DIR}
            -DQt5Multimedia_DIR=${Qt5Multimedia_DIR}
        BUILD_ALWAYS ON
        INSTALL_COMMAND ""
    )
endif()
