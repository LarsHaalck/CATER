add_executable(superglue main.cpp)
target_link_libraries(superglue
    PRIVATE
    image-processing
    cxxopts::cxxopts
    spdlog::spdlog
)
