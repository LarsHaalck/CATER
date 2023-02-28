###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Eigen3 3 REQUIRED)
find_package(cereal REQUIRED)
find_package(mild REQUIRED)

if(${WITH_SUPERGLUE})
    find_package(Torch REQUIRED)
endif()

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

    src/idTranslator.cpp

    # misc
    src/util.cpp
)

if(${WITH_SUPERGLUE})
    list(APPEND SOURCES src/superGlue.cpp)
else()
    list(APPEND SOURCES src/superGlueStub.cpp)
endif()

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "image-processing")
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
        cache_${TYPE}
        progressbar_${TYPE}
        util_${TYPE}
        opencv_core
        opencv_imgcodecs
        opencv_features2d
        opencv_calib3d
        PRIVATE
        opencv_imgproc
        opencv_highgui
        opencv_flann
        spdlog::spdlog
        mild::mild
        cereal::cereal
        io
    )

    if(${WITH_SUPERGLUE})
        target_link_libraries(${CURR_LIB} PUBLIC torch)
    endif()

    if(${OpenMP_CXX_FOUND})
        target_link_libraries(${CURR_LIB}
            PUBLIC
            OpenMP::OpenMP_CXX
        )
    endif()

    install_lib(${CURR_LIB} ON)
endforeach()

install_config(${LIB_NAME})
