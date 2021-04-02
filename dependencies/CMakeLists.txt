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


###########################################################################
#                                 OPTIONS                                 #
###########################################################################
# option(BUILD_GUI "Build the graphical user interface" TRUE)
# option(BUILD_TOOLS "Build tools" TRUE)
# option(WITH_SUPERGLUE "With SuperGlue/SuperPoint support" FALSE)

set(habitrack_dependencies
    eigen3_external
    spdlog_external
    mild_external
    ceres_external
    cereal_external
    cxxopts_external
)

add_library(dummy INTERFACE)
add_dependencies(dummy eigen3_external)

configure_file(cmake_habitrack.sh.in cmake_habitrack.sh)

# include(ExternalProject)
# ExternalProject_Add(habitrack
#     DEPENDS
#         ${habitrack_dependencies}
#     SOURCE_DIR
#         ${PROJECT_SOURCE_DIR}/..
#     CMAKE_ARGS
#         -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#         -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
#         -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
#         -DEigen3_DIR=${Eigen3_DIR}
#         -Dspdlog_DIR=${spdlog_DIR}
#         -Dmild_DIR=${mild_DIR}
#         -DCeres_DIR=${Ceres_DIR}
#         -Dcereal_DIR=${cereal_DIR}
#         -Dcxxopts_DIR=${cxxopts_DIR}
#     BUILD_ALWAYS ON
#     INSTALL_COMMAND ""
# )
