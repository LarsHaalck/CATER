find_package(cxxopts 2.2.1 CONFIG QUIET)

if(TARGET cxxopts::cxxopts)
    message(STATUS "Found cxxopts version")
    add_library(cxxopts_external INTERFACE) # dummy
else()
    message(STATUS "cxxopts could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        cxxopts_external
        GIT_REPOSITORY https://github.com/jarro2783/cxxopts
        GIT_TAG dd45a0801c99d62109aaa29f8c410ba8def2fbf2
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
