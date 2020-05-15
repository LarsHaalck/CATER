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
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
