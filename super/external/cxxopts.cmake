# cxxopts does not export version correctly
find_package(cxxopts 2.2.1 CONFIG QUIET)

if(TARGET cxxopts::cxxopts)
    get_property(loc TARGET cxxopts::cxxopts PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found cxxopts version ${PACKAGE_VERSION} at ${loc}")
    add_library(cxxopts_external INTERFACE)
else()
    message(STATUS "cxxopts could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        cxxopts_external
        GIT_REPOSITORY https://github.com/jarro2783/cxxopts
        GIT_SHALLOW TRUE
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(cxxopts_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/cxxopts)
endif()
