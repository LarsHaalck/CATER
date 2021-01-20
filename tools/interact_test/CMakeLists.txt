find_package(cxxopts REQUIRED)

add_executable(interact_test main.cpp kde.cpp)
target_include_directories(interact_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(interact_test
    PUBLIC
    image-processing
    habitrack
    cxxopts::cxxopts
    spdlog::spdlog
)

add_executable(main main2.cpp kde.cpp)
target_include_directories(main
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
