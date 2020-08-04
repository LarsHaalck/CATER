find_package(cxxopts REQUIRED)

add_executable(match_trafo_exporter main.cpp)
target_link_libraries(match_trafo_exporter
    habitrack
    panorama
    cxxopts::cxxopts
)
