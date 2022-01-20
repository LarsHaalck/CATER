###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "io")
add_library(${LIB_NAME} INTERFACE ${SOURCES})

target_include_directories(${LIB_NAME}
    INTERFACE
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install_lib(${LIB_NAME} ON)
