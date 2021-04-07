find_package(Eigen3 3.3 CONFIG QUIET)

if(TARGET Eigen3::Eigen)
    get_property(loc TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found Eigen3 version ${EIGEN3_VERSION_STRING} at ${loc}")
    add_library(eigen3_external INTERFACE)
else()
    message(STATUS "Eigen3 could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        eigen3_external
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen
        GIT_TAG 3.3.9
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
    set(Eigen3_DIR ${STAGED_INSTALL_PREFIX}/share/eigen3/ceres)
endif()
