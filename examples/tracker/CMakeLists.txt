find_package(cxxopts REQUIRED)

add_executable(tracker main.cpp)
target_link_libraries(tracker
    image-processing
    habitrack
    cxxopts::cxxopts
    spdlog::spdlog
)
