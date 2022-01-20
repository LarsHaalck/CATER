###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)

find_package(OpenCL QUIET)
if(${OpenCL_FOUND})
    message(STATUS "Found OpenCL building tracker with OpenCL support")
endif()

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/unaries.cpp
    src/manualUnaries.cpp
    src/interpTracker.cpp
    src/detections.cpp
    src/tracker.cpp
    src/trackerChunked.cpp
)

if(${OpenCL_FOUND})
    list(APPEND SOURCES src/trackerOpenCL.cpp)
else()
    list(APPEND SOURCES src/trackerContinous.cpp)
endif()

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "tracker")
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${LIB_NAME}
    PUBLIC
    image-processing
    PRIVATE
    cereal
    io
    progressbar
    spdlog::spdlog
    util
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

if(${OpenCL_FOUND})
    target_compile_definitions(${LIB_NAME} PRIVATE HAS_OPENCL)
    target_link_libraries(${LIB_NAME}
        PRIVATE
        OpenCL::OpenCL
    )
endif()

install_lib(${LIB_NAME} ON)
