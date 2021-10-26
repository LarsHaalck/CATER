###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/progressStatusBar.cpp
)

set(GUI_HEADERS
    include/gui/progressStatusBar.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(LIB_NAME common-gui)

# headers need to be added for automoc
add_library(${LIB_NAME} ${SOURCES} ${GUI_HEADERS})
set_target_properties(${LIB_NAME} PROPERTIES AUTOMOC ON)

target_include_directories(${LIB_NAME} PUBLIC include)
target_link_libraries(${LIB_NAME}
    PRIVATE
    Qt6::Widgets
    progressbar
)

