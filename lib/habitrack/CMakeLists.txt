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
    src/detections.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "habitrack")
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${LIB_NAME}
    PUBLIC
    image-processing
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
        spdlog::spdlog
        cereal
        progressbar
        io
    )
endif()
