add_executable(opencv_stitcher main.cpp)
target_link_libraries(opencv_stitcher
    PRIVATE
    util_STATIC
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    opencv_stitching
    spdlog::spdlog
)
