###########################################################################
#                           DEPENDENCIES                                  #
###########################################################################
add_subdirectory(external/cereal)
add_subdirectory(external/mild)

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    main.cpp
    src/cache.cpp
    src/imageCache.cpp
    src/imageContainer.cpp
    src/baseDecorator.cpp
    src/resizeDecorator.cpp
    src/grayDecorator.cpp
    src/specialGrayDecorator.cpp
    src/featureContainer.cpp
    src/unknownFeatureType.cpp
)


###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "lib${PROJECT_NAME}")
add_executable(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${LIB_NAME}
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_xfeatures2d
    opencv_features2d
    opencv_calib3d
    opencv_flann
    opencv_stitching
    opencv_imgcodecs
    cereal
    mild
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${LIB_NAME}
        OpenMP::OpenMP_CXX
    )
endif()

# only for profiling
target_link_libraries(${LIB_NAME}
    ${CMAKE_DL_LIBS}
)
