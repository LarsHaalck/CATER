###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "version")
add_library(${LIB_NAME} INTERFACE ${SOURCES})

configure_file(include/version/version.h.in include/version/version.h)

target_include_directories(${LIB_NAME}
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

install(TARGETS ${LIB_NAME}
    EXPORT habitrack-targets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include DESTINATION .)
