###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Qt5Widgets REQUIRED)

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
    src/labelEditor.cpp
    src/keyDialog.cpp
    src/keyDelegate.cpp
    src/colorDelegate.cpp
)

set(GUI_HEADERS
    include/gui/labelEditor.h
    include/gui/keyDialog.h
    include/gui/keyDelegate.h
    include/gui/colorDelegate.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME label)

# headers need to be added for automoc
add_executable(${EXE_NAME} ${SOURCES} ${GUI_HEADERS})
set_target_properties(${EXE_NAME} PROPERTIES AUTOMOC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTOUIC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTORCC ON)

target_include_directories(${EXE_NAME} PUBLIC include)
target_link_libraries(${EXE_NAME}
    PRIVATE
    Qt5::Widgets
    spdlog::spdlog
)
