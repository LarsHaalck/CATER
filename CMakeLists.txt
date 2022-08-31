cmake_minimum_required(VERSION 3.8)
project(habitrack VERSION 0.1)

###########################################################################
#                            GLOBAL SETTINGS                              #
###########################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(compile_options)
include(habitrack)

###########################################################################
#                                 OPTIONS                                 #
###########################################################################
option(BUILD_GUI "Build the graphical user interface" TRUE)
option(BUILD_TOOLS "Build tools" TRUE)
option(WITH_SUPERGLUE "With SuperGlue/SuperPoint support" FALSE)

###########################################################################
#                       PACKAGES / SUBDIRS                                #
###########################################################################
find_package(OpenCV 4 REQUIRED)
find_package(spdlog 1 REQUIRED)
find_package(OpenMP QUIET)

if(NOT ${OpenMP_CXX_FOUND})
    message(WARNING "Could not find OpenMP; building without...")
endif()

if(OpenCV_VERSION VERSION_LESS "3.0")
    message(FATAL_ERROR "OpenCV version should be >= 3.0")
endif()


add_subdirectory(lib)
add_subdirectory(ui)

if(${BUILD_TOOLS})
    add_subdirectory(tools)
endif()

install_logic()
