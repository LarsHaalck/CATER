project(Habitrack)
cmake_minimum_required(VERSION 3.5)

###########################################################################
#                            GLOBAL SETTINGS                              #
###########################################################################
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# enable highest error level and optimizations
if(MSVC)
    add_compile_options("/W4" "$<$<CONFIG:RELEASE>:/O2>")
else()
    add_compile_options(
        "-Wall"
        "-Wextra"
        "-pedantic"
        "-Wstrict-aliasing"
        "-pedantic"
        "-fmax-errors=5"
        "-Werror"
        "-Wunreachable-code"
        "-Wcast-align"
        "-Wcast-qual"
        "-Wctor-dtor-privacy"
        "-Wdisabled-optimization"
        "-Wformat=2"
        "-Winit-self"
        "-Wlogical-op"
        "-Wmissing-include-dirs"
        "-Wold-style-cast"
        "-Woverloaded-virtual"
        "-Wredundant-decls"
        "-Wshadow"
        "-Wsign-promo"
        "-Wstrict-null-sentinel"
        "-Wstrict-overflow=5"
        "-Wswitch-default"
        "-Wundef"
        "-Wno-unused"
        "-Wno-variadic-macros"
        "-Wno-parentheses"
        "$<$<CONFIG:RELEASE>:-O3>")

    # handle clang specifics
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        add_compile_options("-stdlib=libc++")
    endif()
endif()

###########################################################################
#                                DEBUG                                    #
###########################################################################
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options (-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options (-fcolor-diagnostics)
    endif ()
endif ()

###########################################################################
#                              PACKAGES                                   #
###########################################################################
find_package(OpenCV REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenMP)

if(OpenCV_VERSION VERSION_LESS "3.0")
    message(FATAL_ERROR "OpenCV version should be >= 3.0")
endif()


###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
    src/gui/mainWindow.cpp
    src/gui/trackerGraphicsView.cpp
    src/gui/trackerScene.cpp
)

set(GUI_HEADERS
    include/gui/mainWindow.h
    include/gui/trackerGraphicsView.h
    include/gui/trackerScene.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################

# headers need to be added for automoc
add_executable(${PROJECT_NAME} ${SOURCES} ${GUI_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOUIC ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_highgui
    opencv_videoio
    opencv_imgproc
    opencv_xfeatures2d
    opencv_features2d
    opencv_optflow
    opencv_calib3d
    opencv_video
    opencv_flann
    opencv_stitching
    opencv_imgcodecs
    Qt5::Widgets
    OpenMP::OpenMP_CXX
)
