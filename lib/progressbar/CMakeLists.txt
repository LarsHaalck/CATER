###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/progressBar.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "progressbar")
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install_lib(${LIB_NAME} ON)
