add_executable(keyframes_to_image_list main.cpp)
target_link_libraries(keyframes_to_image_list
    PRIVATE
    panorama_STATIC
    cxxopts::cxxopts
)
