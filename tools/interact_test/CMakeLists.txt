add_executable(interact_test main.cpp kde.cpp)
target_include_directories(interact_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(interact_test
    PUBLIC
    image-processing
    tracker
    cxxopts::cxxopts
    spdlog::spdlog
)

add_executable(interact_test2 main2.cpp kde.cpp)
target_include_directories(interact_test2
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(interact_test2
    PUBLIC
    image-processing
    tracker
    cxxopts::cxxopts
    spdlog::spdlog
)
