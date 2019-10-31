add_executable(lc main.cpp)

target_link_libraries(lc
    libHabitrack
)
# target_include_directories(${LIB_NAME}
#     PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src
# )

# TODO: link agains eigen::eigen later
# this is only for prototyping because of Eigen warnings
# target_include_directories(${LIB_NAME} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

# target_link_libraries(${LIB_NAME}
#     PRIVATE
#     opencv_core
#     opencv_highgui
#     opencv_imgproc
#     opencv_xfeatures2d
#     opencv_features2d
#     opencv_calib3d
#     opencv_flann
#     opencv_stitching
#     opencv_imgcodecs
#     cereal::cereal
#     mild::mild
#     ceres
#     Eigen3::Eigen
# )

# if(${OpenMP_CXX_FOUND})
#     target_link_libraries(${LIB_NAME}
#         PRIVATE
#         OpenMP::OpenMP_CXX
#     )
# endif()

# # only for profiling
# target_link_libraries(${LIB_NAME} PRIVATE
#     ${CMAKE_DL_LIBS}
# )
