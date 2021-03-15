find_package(cxxopts REQUIRED)

add_executable(single main.cpp)
target_link_libraries(single
    PRIVATE
    tracker
    panorama
    cxxopts::cxxopts
    spdlog::spdlog
)
