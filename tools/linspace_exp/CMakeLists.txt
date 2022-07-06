add_executable(linspace_exp main.cpp)
target_include_directories(linspace_exp
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(linspace_exp
    PUBLIC
    image-processing
    tracker
    util
    cxxopts::cxxopts
    spdlog::spdlog
)
