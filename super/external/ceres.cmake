find_package(Ceres 2.0 CONFIG QUIET)

if(TARGET Ceres)
    message(STATUS "Found ceres version")
    add_library(ceres_external INTERFACE) # dummy
else()
    message(STATUS "ceres could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        ceres_external
        GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver/
        GIT_TAG 2.0.0
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DEIGENSPARSE=ON
          -DBUILD_SHARED_LIBS=ON
          -DBUILD_TESTING=OFF
          -DBUILD_EXAMPLES=OFF
          -DBUILD_BENCHMARKS=OFF
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(Ceres_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/ceres)
endif()
