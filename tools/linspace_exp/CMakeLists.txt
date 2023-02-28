add_executable(linspace_exp main.cpp)
target_include_directories(linspace_exp
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(linspace_exp
    PUBLIC
    image-processing_STATIC
    tracker_STATIC
    util_STATIC
    cxxopts::cxxopts
    spdlog::spdlog
)
