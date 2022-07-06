add_executable(interact_test_reverse main_reverse.cpp kde.cpp)
add_executable(interact_test_eq main_eq.cpp)
add_executable(interact_test_max main_max.cpp)

set(prg_names interact_test_reverse interact_test_eq interact_test_max)
foreach(prg IN LISTS prg_names)
    target_include_directories(${prg}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(${prg}
        PUBLIC
        image-processing
        tracker
        model
        cxxopts::cxxopts
        spdlog::spdlog
    )
endforeach()

