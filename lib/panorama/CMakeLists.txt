###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Ceres 2.0 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(cereal REQUIRED)

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/keyFrames.cpp
    src/keyFrameRecommender.cpp
    src/interExhaustiveRecommender.cpp
    src/panoramaStitcher.cpp
    src/settings.cpp
    src/panoramaEngine.cpp
    src/gpsInterpolator.cpp

    src/metrics.cpp
    src/affinityGlobalOptimizer.cpp
    src/homographyGlobalOptimizer.cpp
    src/similarityGlobalOptimizer.cpp
    src/isometryGlobalOptimizer.cpp
    src/similarityGPSOptimizer.cpp

    src/transitions.cpp
    src/mst.cpp
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
    PUBLIC
    cereal
    ceres
    io
    opencv_stitching
    progressbar
    spdlog::spdlog
    util
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        PUBLIC
        OpenMP::OpenMP_CXX
    )
endif()

install_lib(${LIB_NAME} ON)
install_config(${LIB_NAME})
