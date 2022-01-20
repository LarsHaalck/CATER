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
    tracker
    util
    PUBLIC
    spdlog::spdlog
    version
    io
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        PUBLIC
        OpenMP::OpenMP_CXX
    )
endif()

install_lib(${LIB_NAME} ON)
