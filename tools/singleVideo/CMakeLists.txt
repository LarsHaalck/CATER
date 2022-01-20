add_executable(single main.cpp)
target_link_libraries(single
    PUBLIC
    tracker
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
