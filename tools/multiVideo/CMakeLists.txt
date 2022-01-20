add_executable(multi main.cpp)
target_link_libraries(multi
    PUBLIC
    tracker
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
