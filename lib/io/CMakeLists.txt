###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "io")
add_library(${LIB_NAME} INTERFACE ${SOURCES})

target_include_directories(${LIB_NAME}
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
