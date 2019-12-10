project(Habitrack)
cmake_minimum_required(VERSION 3.5)

###########################################################################
#                            GLOBAL SETTINGS                              #
###########################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

option(BUILD_GUI "Build the graphical user interface" FALSE)

# enable highest error level and optimizations
if(MSVC)
    add_compile_options("/W4" "$<$<CONFIG:RELEASE>:/O2>")
else()
    add_compile_options(
        "-g"
        "-Wall"
        "-Wextra"
        "-pedantic"
        "-Wstrict-aliasing"
        "-pedantic"
        "-Wunreachable-code"
        "-Wcast-align"
        "-Wcast-qual"
        "-Wctor-dtor-privacy"
        "-Wdisabled-optimization"
        "-Wformat=2"
        "-Winit-self"
        "-Wmissing-include-dirs"
        "-Wold-style-cast"
        "-Woverloaded-virtual"
        "-Wredundant-decls"
        "-Wshadow"
        "-Wsign-promo"
        "-Wstrict-overflow=5"
        "-Wswitch-default"
        "-Wundef"
        "$<$<CONFIG:RELEASE>:-O3>")
endif()

###########################################################################
#                          !! DEBUG DELETE LATER!!                        #
###########################################################################
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
   add_compile_options (-fdiagnostics-color=always)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
   add_compile_options (-fcolor-diagnostics)
endif ()

###########################################################################
#                       PACKAGES / SUBDIRS                                #
###########################################################################
find_package(OpenCV REQUIRED)
find_package(OpenMP QUIET)
find_package(Eigen3 REQUIRED)
find_package(cxxopts REQUIRED)

if(NOT ${OpenMP_CXX_FOUND})
    message(STATUS "Could not find OpenMP; building without...")
endif()

if(OpenCV_VERSION VERSION_LESS "3.0")
    message(FATAL_ERROR "OpenCV version should be >= 3.0")
endif()

if(${BUILD_GUI})
    find_package(Qt5Widgets REQUIRED)
    add_subdirectory(gui)
endif()

add_subdirectory(lib)
add_subdirectory(singleVideo)
add_subdirectory(multiVideo)
