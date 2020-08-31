find_package(cxxopts REQUIRED)

add_executable(single main.cpp)
target_link_libraries(single
    PRIVATE
    habitrack
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
