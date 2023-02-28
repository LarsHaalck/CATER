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
foreach(TYPE IN LISTS LIBS)
    set(CURR_LIB "${LIB_NAME}_${TYPE}")
    add_library(${CURR_LIB} ${TYPE} ${SOURCES})

    target_include_directories(${CURR_LIB}
        PUBLIC
            $<INSTALL_INTERFACE:include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    install_lib(${CURR_LIB} ON)
endforeach()
