###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)
find_package(mild REQUIRED)
###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    # image container related
    src/baseImageContainer.cpp
    src/images.cpp
    src/imageAggregator.cpp
    src/imageCache.cpp
    src/pairwiseImageCache.cpp

    # feature container related
    src/baseFeatureContainer.cpp
    src/features.cpp
    # src/featureAggregator.cpp
    src/pairwiseFeatureCache.cpp
    src/featureCache.cpp
    src/descriptorCache.cpp
    src/pairwiseDescriptorCache.cpp
    src/unknownFeatureType.cpp

    # matches container related
    src/geometricType.cpp
    src/matches.cpp
    src/unknownGeometricType.cpp
    src/ransac.cpp
    src/isometry.cpp
    src/transformation.cpp
    # src/mildRecommender.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "image-processing")
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${LIB_NAME}
    PUBLIC
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_xfeatures2d
    opencv_features2d
    opencv_flann
    opencv_imgcodecs
    cache
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
        spdlog::spdlog
        mild::mild
        cereal
        progressbar
        io
    )
endif()
