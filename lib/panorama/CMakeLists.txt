###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Ceres 2.1 REQUIRED)
find_package(Eigen3 3 REQUIRED)
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
        Ceres::ceres
        Eigen3::Eigen
        PRIVATE
        util_${TYPE}
        progressbar_${TYPE}
        io
        cereal::cereal
        opencv_stitching
        spdlog::spdlog
    )

    if(${OpenMP_CXX_FOUND})
        target_link_libraries(${CURR_LIB}
            PUBLIC
            OpenMP::OpenMP_CXX
        )
    endif()

    install_lib(${CURR_LIB} ON)
endforeach()
install_config(${LIB_NAME})
