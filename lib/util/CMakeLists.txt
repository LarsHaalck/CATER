###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/threadPool.cpp
    src/log.cpp
)
###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME "util")
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

    target_link_libraries(${CURR_LIB}
        PUBLIC
        spdlog::spdlog
    )

    install_lib(${CURR_LIB} ON)
endforeach()
