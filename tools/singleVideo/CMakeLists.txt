add_executable(single main.cpp)
target_link_libraries(single
    PUBLIC
    tracker_STATIC
    panorama_STATIC
    cxxopts::cxxopts
    spdlog::spdlog
)
