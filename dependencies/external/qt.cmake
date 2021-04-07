find_package(Qt5Widgets 5.10 CONFIG QUIET)
find_package(Qt5Concurrent 5.10 CONFIG QUIET)
find_package(Qt5Multimedia 5.10 CONFIG QUIET)

if(TARGET Qt5::Widgets AND TARGET Qt5::Concurrent)
    get_property(loc TARGET Qt5::Widgets PROPERTY LOCATION)
    message(STATUS "Found Qt5Widgets version ${PACKAGE_VERSION} at ${loc}")

    get_property(loc TARGET Qt5::Concurrent PROPERTY LOCATION)
    message(STATUS "Found Qt5Concurrent version ${PACKAGE_VERSION} at ${loc}")

    add_library(qt5base_external INTERFACE)
else()
    message(STATUS "Qt5Widgets or Qt5Concurrent could not be located. Building from source...")
    find_program(MAKE_EXE NAMES gmake nmake make)

    include(ExternalProject)
    ExternalProject_Add(
        qt5base_external
        URL https://download.qt.io/official_releases/qt/5.15/5.15.2/submodules/qtbase-everywhere-src-5.15.2.tar.xz
        URL_MD5 0eb522ff6c2194f9690cbdcd5b33e803
        CONFIGURE_COMMAND
            <SOURCE_DIR>/configure
            -prefix ${STAGED_INSTALL_PREFIX}
            -opensource
            -confirm-license
            -nomake examples
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(Qt5Core_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt5Core)
    set(Qt5Concurrent_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt5Concurrent)
    set(Qt5Widgets_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt5Widgets)
    set(CMAKE_INSTALL_RPATH ${STAGED_INSTALL_PREFIX}/lib)
endif()

if(TARGET Qt5::Multimedia)
    get_property(loc TARGET Qt5::Multimedia PROPERTY LOCATION)
    message(STATUS "Found Qt5Multimedia version ${PACKAGE_VERSION} at ${loc}")
    add_library(qt5multimedia_external INTERFACE)
else()
    message(STATUS "Qt5Multimedia could not be located. Building from source...")

    find_package(Qt5Core 5.10 CONFIG QUIET)
    if(TARGET Qt5::qmake)
        set(qmake $<TARGET_FILE:Qt5::qmake>)
    else()
        set(qmake ${STAGED_INSTALL_PREFIX}/bin/qmake)
    endif()

    include(ExternalProject)
    ExternalProject_Add(
        qt5multimedia_external
        DEPENDS qt5base_external
        URL https://download.qt.io/official_releases/qt/5.15/5.15.2/submodules/qtmultimedia-everywhere-src-5.15.2.tar.xz
        URL_MD5 591e3c3322742eaf76bc6f91dce59e42
        CONFIGURE_COMMAND
            ${STAGED_INSTALL_PREFIX}/bin/qmake <SOURCE_DIR>
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(Qt5Multimedia_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt5Multimedia)
    set(CMAKE_INSTALL_RPATH ${STAGED_INSTALL_PREFIX}/lib)
endif()
