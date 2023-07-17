###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
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
    include/mainWindow.h
    include/preferencesDialog.h
    include/trackerScene.h
    include/trackerGraphicsView.h
    include/unaryScene.h
    include/unaryGraphicsView.h

    # label editor
    include/labelEditor.h
    include/keyDialog.h
    include/keyDelegate.h
    include/colorDelegate.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME cater-gui)

# headers need to be added for automoc
add_executable(${EXE_NAME} ${SOURCES} ${GUI_HEADERS})
set_target_properties(${EXE_NAME} PROPERTIES AUTOMOC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTOUIC ON)
set_target_properties(${EXE_NAME} PROPERTIES AUTORCC ON)

target_include_directories(${EXE_NAME} PUBLIC include)
target_link_libraries(${EXE_NAME}
    PRIVATE
    progressbar_STATIC
    model_STATIC
    util_STATIC
    io
    common-gui
    Qt6::Widgets
    Qt6::Concurrent
    Qt6::Multimedia
    spdlog::spdlog
    cereal::cereal
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${EXE_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

install_bin(${EXE_NAME})
install(FILES resources/cater-gui.desktop DESTINATION share/applications)
install(FILES resources/cater-gui.png
    DESTINATION share/icons/hicolor/128x128/apps)
