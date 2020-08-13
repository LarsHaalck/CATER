find_package(cxxopts REQUIRED)

add_executable(multi main.cpp)
target_link_libraries(multi
    habitrack
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
