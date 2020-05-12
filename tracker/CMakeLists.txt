add_executable(tracker main.cpp)
target_link_libraries(tracker
    libHabitrack
    cxxopts::cxxopts
    spdlog::spdlog
)
