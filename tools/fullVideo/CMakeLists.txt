add_executable(full main.cpp)
target_link_libraries(full
    PRIVATE
    tracker
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
