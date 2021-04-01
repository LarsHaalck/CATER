find_package(cereal 1.3.0 CONFIG QUIET)

if(TARGET cereal)
    message(STATUS "Found cereal version")
    add_library(cereal_external INTERFACE) # dummy
else()
    message(STATUS "cereal could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        cereal_external
        GIT_REPOSITORY https://github.com/USCiLab/cereal
        GIT_TAG v1.3.0
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DSKIP_PORTABILITY_TEST=ON
          -DSKIP_PERFORMANCE_COMPARISON=ON
          -DWITH_WERROR=OFF
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(cereal_DIR ${STAGED_INSTALL_PREFIX}/share/cmake/cereal)
endif()
