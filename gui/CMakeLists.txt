###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
    src/mainWindow.cpp
    src/preferences.cpp
    src/preferencesDialog.cpp
    src/trackerGraphicsView.cpp
    src/trackerScene.cpp
    src/qtOpencvCore.cpp
    src/progressStatusBar.cpp
)

set(GUI_HEADERS
    include/gui/mainWindow.h
    include/gui/preferencesDialog.h
    include/gui/trackerGraphicsView.h
    include/gui/trackerScene.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME habitrack-gui)

# headers need to be added for automoc
add_executable(${EXE_NAME} ${SOURCES} ${GUI_HEADERS})
set_target_properties(${EXE_NAME} PROPERTIES AUTOMOC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTOUIC ON)

target_include_directories(${EXE_NAME} PUBLIC include)

target_link_libraries(${EXE_NAME}
    Qt5::Widgets
    OpenMP::OpenMP_CXX
    habitrack
    spdlog::spdlog
    progressbar
)
