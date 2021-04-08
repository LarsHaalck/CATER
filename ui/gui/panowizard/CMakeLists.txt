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
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Multimedia
    spdlog::spdlog
    progressbar
    habitrack
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
