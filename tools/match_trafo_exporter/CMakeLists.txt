add_executable(match_trafo_exporter main.cpp)
target_link_libraries(match_trafo_exporter
    PRIVATE
    tracker_STATIC
    panorama_STATIC
    cxxopts::cxxopts
)
