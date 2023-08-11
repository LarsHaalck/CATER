###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(cereal REQUIRED)

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME cater-cli)

# headers need to be added for automoc
add_executable(${EXE_NAME} ${SOURCES})

target_include_directories(${EXE_NAME} PUBLIC include)
target_link_libraries(${EXE_NAME}
    PRIVATE
    progressbar_STATIC
    model_STATIC
    util_STATIC
    panorama_STATIC
    io
    spdlog::spdlog
    cereal::cereal
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${EXE_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

install_bin(${EXE_NAME})
