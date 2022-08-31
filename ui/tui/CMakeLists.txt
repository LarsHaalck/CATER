###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
    src/tui.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME habitrack-tui)

# headers need to be added for automoc
add_executable(${EXE_NAME} ${SOURCES})

target_include_directories(${EXE_NAME} PUBLIC include)
target_link_libraries(${EXE_NAME}
    PRIVATE
    spdlog::spdlog
    progressbar
    model
    cereal::cereal
    util
    io
    panorama
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${EXE_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

install_bin(${EXE_NAME})
