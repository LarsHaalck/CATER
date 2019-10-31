###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Ceres REQUIRED)
find_package(cereal REQUIRED)
find_package(mild REQUIRED)
###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    # main.cpp
    # abstract classes
    src/baseCache.cpp
    src/baseDecorator.cpp

    # image container related
    src/baseImageContainer.cpp
    src/imageContainer.cpp
    src/imageAggregator.cpp
    src/imageCache.cpp
    src/resizeDecorator.cpp
    src/grayDecorator.cpp
    src/specialGrayDecorator.cpp

    # feature container
    src/baseFeatureContainer.cpp
    src/featureContainer.cpp
    src/featureAggregator.cpp
    src/featureCache.cpp
    src/descriptorCache.cpp
    src/unknownFeatureType.cpp

    # matches container
    src/geometricType.cpp
    src/matchesContainer.cpp
    src/unknownGeometricType.cpp
    src/basePairwiseCache.cpp
    src/pairwiseDescriptorCache.cpp
    src/pairwiseFeatureCache.cpp
    src/ransac.cpp
    src/isometry.cpp
    src/transformation.cpp

    # key frames
    src/keyFrameSelector.cpp

    # pano
    src/panoramaStitcher.cpp
    src/homographyGlobalOptimizer.cpp
    src/affinityGlobalOptimizer.cpp
    src/similarityGlobalOptimizer.cpp
    src/isometryGlobalOptimizer.cpp
)


###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "libHabitrack")
# add_executable(${LIB_NAME} ${SOURCES} main.cpp)
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# TODO: link agains eigen::eigen later
# this is only for prototyping because of Eigen warnings
# target_include_directories(${LIB_NAME} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${LIB_NAME}
    PUBLIC
    opencv_core
    PRIVATE
    opencv_highgui
    opencv_imgproc
    opencv_xfeatures2d
    opencv_features2d
    opencv_calib3d
    opencv_flann
    opencv_stitching
    opencv_imgcodecs
    cereal::cereal
    mild::mild
    ceres
    Eigen3::Eigen
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

# only for profiling
target_link_libraries(${LIB_NAME} PRIVATE
    ${CMAKE_DL_LIBS}
)
