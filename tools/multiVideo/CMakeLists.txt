find_package(cxxopts REQUIRED)

add_executable(multi main.cpp)
target_link_libraries(multi
    PRIVATE
    habitrack
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
