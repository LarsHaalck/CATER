###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/keyFrames.cpp
    src/keyFrameRecommender.cpp
    src/panoramaStitcher.cpp

    src/metrics.cpp
    src/affinityGlobalOptimizer.cpp
    src/homographyGlobalOptimizer.cpp
    src/similarityGlobalOptimizer.cpp
    src/isometryGlobalOptimizer.cpp

    src/idTranslator.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "panorama")
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
    Eigen3::Eigen
    PRIVATE
    spdlog::spdlog
    cereal
    ceres
    progressbar
    io
    opencv_stitching

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
