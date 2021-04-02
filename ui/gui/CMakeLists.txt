###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Qt5Widgets 5.10 REQUIRED)
find_package(Qt5Concurrent 5.10 REQUIRED)
find_package(Qt5Multimedia 5.10 REQUIRED)
find_package(cereal REQUIRED)

###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
    src/mainWindow.cpp
    src/preferencesDialog.cpp
    src/trackerScene.cpp
    src/trackerGraphicsView.cpp
    src/unaryScene.cpp
    src/unaryGraphicsView.cpp
    src/qtOpencvCore.cpp
    src/progressStatusBar.cpp
    src/imageViewer.cpp
    resources/resources.qrc

    # label editor
    src/labelEditor.cpp
    src/keyDialog.cpp
    src/keyDelegate.cpp
    src/colorDelegate.cpp

    # label
    src/labelConfig.cpp
    src/labeler.cpp
)

set(GUI_HEADERS
    include/gui/mainWindow.h
    include/gui/preferencesDialog.h
    include/gui/trackerScene.h
    include/gui/trackerGraphicsView.h
    include/gui/unaryScene.h
    include/gui/unaryGraphicsView.h
    include/gui/imagesWorker.h
    include/gui/progressStatusBar.h

    # label editor
    include/gui/labelEditor.h
    include/gui/keyDialog.h
    include/gui/keyDelegate.h
    include/gui/colorDelegate.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME habitrack-gui)

# headers need to be added for automoc
add_executable(${EXE_NAME} ${SOURCES} ${GUI_HEADERS})
set_target_properties(${EXE_NAME} PROPERTIES AUTOMOC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTOUIC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTORCC ON)

target_include_directories(${EXE_NAME} PUBLIC include)
target_link_libraries(${EXE_NAME}
    PRIVATE
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Multimedia
    spdlog::spdlog
    progressbar
    habitrack
    cereal
    util
    io
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${EXE_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

install(TARGETS ${EXE_NAME}
    DESTINATION bin
)

install(FILES resources/habitrack-gui.desktop DESTINATION share/applications)
install(FILES resources/habitrack-gui.png
    DESTINATION share/icons/hicolor/128x128/apps)
