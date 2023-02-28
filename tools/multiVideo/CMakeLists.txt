add_executable(multi main.cpp)
target_link_libraries(multi
    PUBLIC
    tracker_STATIC
    panorama_STATIC
    cxxopts::cxxopts
    spdlog::spdlog
)
