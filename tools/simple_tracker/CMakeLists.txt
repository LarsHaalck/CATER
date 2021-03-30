add_executable(simple_tracker main.cpp)
target_link_libraries(simple_tracker
    PUBLIC
    image-processing
    tracker
    cxxopts::cxxopts
    spdlog::spdlog
)
