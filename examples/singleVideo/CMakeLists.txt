find_package(cxxopts REQUIRED)

add_executable(single main.cpp)
target_link_libraries(single
    habitrack
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
