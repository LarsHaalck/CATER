add_executable(superglue main.cpp)
target_link_libraries(superglue
    PUBLIC
    image-processing_STATIC
    cxxopts::cxxopts
    spdlog::spdlog
)
