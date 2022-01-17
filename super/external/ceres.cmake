# TODO add 2.x tag when #669 is released
find_package(Ceres 2.0 CONFIG QUIET)

if(TARGET ceres)
    get_property(loc TARGET ceres PROPERTY LOCATION)
    message(STATUS "Found Ceres version ${PACKAGE_VERSION} at ${loc}")
    add_library(ceres_external INTERFACE) # dummy
else()
    message(STATUS "Ceres could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        ceres_external
        DEPENDS eigen3_external
        GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver/
        GIT_SHALLOW TRUE
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DEigen3_DIR=${Eigen3_DIR}
          -DLIB_SUFFIX="" # needed to remove cases where Ceres_DIR should be in lib64
          -DCMAKE_INSTALL_LIBDIR=lib
          -DEIGENSPARSE=ON
          -DBUILD_SHARED_LIBS=ON
          -DBUILD_TESTING=OFF
          -DBUILD_EXAMPLES=OFF
          -DBUILD_BENCHMARKS=OFF
          -DMINIGLOG=ON
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(Ceres_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/Ceres)
endif()
