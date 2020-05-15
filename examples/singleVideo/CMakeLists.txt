add_executable(single main.cpp)
target_link_libraries(single
    libHabitrack
    cxxopts::cxxopts
)
