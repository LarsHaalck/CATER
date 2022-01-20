###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/baseCache.cpp
    src/basePairwiseCache.cpp
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "cache")
add_library(${LIB_NAME} ${SOURCES})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install_lib(${LIB_NAME} ON)
