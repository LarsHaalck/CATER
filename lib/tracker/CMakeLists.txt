###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)
###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/unaries.cpp
    src/manualUnaries.cpp
    src/tracker.cpp
    src/tracker2.cpp
    src/detections.cpp
)

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

install(TARGETS ${LIB_NAME}
    EXPORT habitrack-targets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION include)