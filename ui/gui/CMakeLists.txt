###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Multimedia QUIET)
find_package(cereal REQUIRED)
find_package(cxxopts REQUIRED)

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
    src/setIO.cpp
    src/imageViewer.cpp
    resources/resources.qrc
)

set(GUI_HEADERS
    include/gui/mainWindow.h
    include/gui/preferencesDialog.h
    include/gui/trackerScene.h
    include/gui/trackerGraphicsView.h
    include/gui/unaryScene.h
    include/gui/unaryGraphicsView.h
    include/gui/imagesWorker.h
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
    OpenMP::OpenMP_CXX
    spdlog::spdlog
    progressbar
    cxxopts::cxxopts
    habitrack
)

if(${QT5_MULTIMEDIA_FOUND})
    target_link_libraries(${EXE_NAME}
        PRIVATE
        Qt5::Multimedia
    )
    add_compile_definitions(WITH_QT5_MULTIMEDIA)
endif()

install(TARGETS ${EXE_NAME}
    DESTINATION bin
)

install(FILES resources/habitrack-gui.desktop DESTINATION share/applications)
install(FILES resources/habitrack-gui.png
    DESTINATION share/icons/hicolor/128x128/apps)
