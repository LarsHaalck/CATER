find_package(cxxopts REQUIRED)

add_executable(tracker main.cpp)
target_link_libraries(tracker
    PUBLIC
    image-processing
    habitrack
    cxxopts::cxxopts
    spdlog::spdlog
)
