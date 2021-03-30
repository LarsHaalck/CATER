add_executable(multi main.cpp)
target_link_libraries(multi
    PRIVATE
    tracker
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
