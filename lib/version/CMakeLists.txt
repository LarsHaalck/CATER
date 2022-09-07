###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "version")
add_library(${LIB_NAME} INTERFACE ${SOURCES})

configure_file(include/cater/version/version.h.in include/cater/version/version.h)

target_include_directories(${LIB_NAME}
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

install_lib(${LIB_NAME} ON)
