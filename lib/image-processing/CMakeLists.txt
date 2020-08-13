###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)
find_package(mild REQUIRED)
find_package(Eigen3 REQUIRED)
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
    src/featureAggregator.cpp
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
    src/mildRecommender.cpp

    src/util.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "image-processing")
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
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_xfeatures2d
    opencv_features2d
    opencv_flann
    opencv_imgcodecs
    cache
    progressbar
    PRIVATE
    spdlog::spdlog
    mild::mild
    cereal
    io
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

