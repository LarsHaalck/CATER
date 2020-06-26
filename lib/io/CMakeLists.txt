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

install(TARGETS ${LIB_NAME}
    EXPORT habitrack-targets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION include)
