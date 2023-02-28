add_executable(full main.cpp)
target_link_libraries(full
    PRIVATE
    tracker_STATIC
    panorama_STATIC
    cxxopts::cxxopts
    spdlog::spdlog
)
