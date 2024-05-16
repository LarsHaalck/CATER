cmake_minimum_required(VERSION 3.8)
project(cater-superbuild)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set_property(DIRECTORY PROPERTY EP_BASE ${CMAKE_BINARY_DIR}/external)


set(STAGED_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/stage)
message(STATUS "${PROJECT_NAME} staged install: ${STAGED_INSTALL_PREFIX}")

include(external/eigen3)
include(external/opencv)
include(external/spdlog)
include(external/mild)
include(external/ceres)
include(external/cereal)
include(external/cxxopts)
include(external/qt)

option(BUILD_DEPS_ONLY "Build only dependencies and nothing else (used for development)" FALSE)
option(INSTALL_CATER "Install Cater (only possible when BUILD_DEPS_ONLY=FALSE)" FALSE)

set(cater_dependencies
    eigen3_external
    spdlog_external
    mild_external
    ceres_external
    cereal_external
    cxxopts_external
    opencv_external
    qt6base_external
    qt6multimedia_external
)


if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "install" CACHE PATH "..." FORCE)
endif()

if (${BUILD_DEPS_ONLY})
    add_library(dummy INTERFACE)
    add_dependencies(dummy ${cater_dependencies})
    configure_file(cmake_cater.in cmake_cater)
else ()
    include(ExternalProject)
    ExternalProject_Add(cater
        DEPENDS
            ${cater_dependencies}
        SOURCE_DIR
            ${PROJECT_SOURCE_DIR}/..
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=$<IF:$<BOOL:${CMAKE_BUILD_TYPE}>,${CMAKE_BUILD_TYPE},Release>
            -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DBUILD_GUI=$<IF:$<BOOL:${BUILD_GUI}>,${BUILD_GUI},OFF>
            -DBUILD_TOOLS=$<IF:$<BOOL:${BUILD_TOOLS}>,${BUILD_TOOLS},OFF>
            -DWITH_SUPERGLUE=$<IF:$<BOOL:${WITH_SUPERGLUE}>,${WITH_SUPERGLUE},OFF>
            -DEigen3_DIR=${Eigen3_DIR}
            -Dspdlog_DIR=${spdlog_DIR}
            -Dmild_DIR=${mild_DIR}
            -DCeres_DIR=${Ceres_DIR}
            -Dcereal_DIR=${cereal_DIR}
            -Dcxxopts_DIR=${cxxopts_DIR}
            -DOpenCV_DIR=${OpenCV_DIR}
            -DQt6CoreTools_DIR=${Qt6CoreTools_DIR}
            -DQt6Concurrent_DIR=${Qt6Concurrent_DIR}
            -DQt6Widgets_DIR=${Qt6Widgets_DIR}
            -DQt6WidgetsTools_DIR=${Qt6WidgetsTools_DIR}
            -DQt6GuiTools_DIR=${Qt6GuiTools_DIR}
            -DQt6Multimedia_DIR=${Qt6Multimedia_DIR}
            -DQt6DBusTools_DIR=${Qt6DBusTools_DIR}
            -DCMAKE_INSTALL_RPATH=${CMAKE_INSTALL_RPATH}
        BUILD_ALWAYS ON
        INSTALL_COMMAND ""
    )
endif()
