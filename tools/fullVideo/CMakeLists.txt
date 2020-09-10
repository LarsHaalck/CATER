find_package(cxxopts REQUIRED)

add_executable(full main.cpp)
target_link_libraries(full
    PRIVATE
    habitrack
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
