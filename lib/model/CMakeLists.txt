###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/preferences.cpp
    src/model.cpp
    src/resultsIO.cpp
    src/imageViewer.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "model")
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
        tracker_${TYPE}
        util_${TYPE}
        PRIVATE
        spdlog::spdlog
        version
        io
        opencv_videoio
    )

    if(${OpenMP_CXX_FOUND})
        target_link_libraries(${CURR_LIB}
            PUBLIC
            OpenMP::OpenMP_CXX
        )
    endif()

    install_lib(${CURR_LIB} ON)
endforeach()
