add_executable(simple_tracker main.cpp)
target_link_libraries(simple_tracker
    PRIVATE
    image-processing_STATIC
    tracker_STATIC
    cxxopts::cxxopts
    spdlog::spdlog
)
