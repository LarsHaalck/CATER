###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal 1.3 REQUIRED)

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
foreach(TYPE IN LISTS LIBS)
    set(CURR_LIB "${LIB_NAME}_${TYPE}")
    add_library(${CURR_LIB} ${TYPE} ${SOURCES})

    target_include_directories(${CURR_LIB}
        PUBLIC
            $<INSTALL_INTERFACE:include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(${CURR_LIB}
        PUBLIC
        image-processing_${TYPE}
        PRIVATE
        progressbar_${TYPE}
        util_${TYPE}
        cereal::cereal
        io
        spdlog::spdlog
    )

    if(${OpenMP_CXX_FOUND})
        target_link_libraries(${CURR_LIB}
            PUBLIC
            OpenMP::OpenMP_CXX
        )
    endif()

    if(${OpenCL_FOUND})
        target_compile_definitions(${CURR_LIB} PRIVATE HAS_OPENCL)
        target_link_libraries(${CURR_LIB}
            PUBLIC
            OpenCL::OpenCL
        )
    endif()
    install_lib(${CURR_LIB} ON)
endforeach()

install_config(${LIB_NAME})
