###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    # image container related
    src/progressBar.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "progressbar")
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
