###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/gui/main.cpp
    src/gui/mainWindow.cpp
    src/gui/preferencesDialog.cpp
    src/gui/trackerGraphicsView.cpp
    src/gui/trackerScene.cpp
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

# headers need to be added for automoc
add_executable(${PROJECT_NAME} ${SOURCES} ${GUI_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOUIC ON)

target_link_libraries(${PROJECT_NAME}
    opencv_highgui
    opencv_videoio
    opencv_imgproc
    opencv_xfeatures2d
    opencv_features2d
    opencv_optflow
    opencv_calib3d
    opencv_video
    opencv_flann
    opencv_stitching
    opencv_imgcodecs
    Qt5::Widgets
    OpenMP::OpenMP_CXX
)
