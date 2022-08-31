find_package(Qt6Widgets 6 CONFIG QUIET)
find_package(Qt6Concurrent 6 CONFIG QUIET)
find_package(Qt6Multimedia 6 CONFIG QUIET)

if(TARGET Qt6::Widgets AND TARGET Qt6::Concurrent AND TARGET Qt6::Multimedia)
    get_property(loc TARGET Qt6::Widgets PROPERTY LOCATION)
    message(STATUS "Found Qt6Widgets at ${loc}")
    get_property(loc TARGET Qt6::Concurrent PROPERTY LOCATION)
    message(STATUS "Found Qt6Concurrent at ${loc}")
    get_property(loc TARGET Qt6::Multimedia PROPERTY LOCATION)
    message(STATUS "Found Qt6Multimedia at ${loc}")

    add_library(qt6base_external INTERFACE)
    add_library(qt6multimedia_external INTERFACE)
else()
    message(STATUS "Qt6Widgets, Qt6Concurrent or Qt6Multimedia could not be located. Building from source...")
    find_program(MAKE_EXE NAMES gmake nmake make)

    include(ExternalProject)
    ExternalProject_Add(
        qt6base_external
        URL https://download.qt.io/official_releases/qt/6.3/6.3.1/submodules/qtbase-everywhere-src-6.3.1.tar.xz
        URL_MD5 470bce74df4895f4c45ca335b06eaa0a
        CONFIGURE_COMMAND
            <SOURCE_DIR>/configure
            -prefix ${STAGED_INSTALL_PREFIX}
            -cmake-generator ${CMAKE_GENERATOR}
            -no-opengl
            -opensource
            -confirm-license
            -nomake examples
        BUILD_COMMAND cmake --build .
        INSTALL_COMMAND cmake --install .
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )

    # set(qmake ${STAGED_INSTALL_PREFIX}/bin/qmake)
    set(Qt6Core_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6Core)
    set(Qt6CoreTools_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6CoreTools)
    set(Qt6Concurrent_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6Concurrent)
    set(Qt6Widgets_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6Widgets)
    set(Qt6WidgetsTools_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6WidgetsTools)
    set(Qt6GuiTools_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6GuiTools)
    ExternalProject_Add(
        qt6shadertools_external
        DEPENDS qt6base_external
        URL https://download.qt.io/official_releases/qt/6.3/6.3.1/submodules/qtshadertools-everywhere-src-6.3.1.tar.xz
        URL_MD5 bbc8a6003407f82fd9dd887899393913
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DQt6Concurrent_DIR=${Qt6Concurrent_DIR}
            -DQt6Widgets_DIR=${Qt6Widgets_DIR}
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )

    ExternalProject_Add(
        qt6multimedia_external
        DEPENDS qt6base_external qt6shadertools_external
        URL https://download.qt.io/official_releases/qt/6.3/6.3.1/submodules/qtmultimedia-everywhere-src-6.3.1.tar.xz
        URL_MD5 1981c400b0790849106e2e98ef76562e
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DQt6Concurrent_DIR=${Qt6Concurrent_DIR}
            -DQt6Widgets_DIR=${Qt6Widgets_DIR}
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(Qt6Multimedia_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Qt6Multimedia)
    set(CMAKE_INSTALL_RPATH ${STAGED_INSTALL_PREFIX}/lib)
endif()
