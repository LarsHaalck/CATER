add_executable(superglue main.cpp)
target_link_libraries(superglue
    PUBLIC
    image-processing
    cxxopts::cxxopts
    spdlog::spdlog
)
