###########################################################################
#                                FILES                                    #
###########################################################################
set(SOURCES
    src/main.cpp
    src/panoWizard.cpp
)

set(GUI_HEADERS
    include/panoWizard.h
)

###########################################################################
#                               TARGETS                                   #
###########################################################################
set(EXE_NAME panowizard)

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
    panorama_STATIC
    util_STATIC
    io
    common-gui
    Qt6::Widgets
    Qt6::Concurrent
    Qt6::Multimedia
    spdlog::spdlog
)

if(${OpenMP_CXX_FOUND})
    target_link_libraries(${EXE_NAME}
        PRIVATE
        OpenMP::OpenMP_CXX
    )
endif()

install_bin(${EXE_NAME})
install(FILES resources/panowizard.desktop DESTINATION share/applications)
install(FILES resources/panowizard.png
    DESTINATION share/icons/hicolor/128x128/apps)
