find_package(fitpackpp CONFIG QUIET)

if(TARGET fitpackpp::fitpackpp)
    get_property(loc TARGET fitpackpp::fitpackpp PROPERTY LOCATION)
    message(STATUS "Found fitpackpp at ${loc}")
    add_library(fitpackpp_external INTERFACE)
else()
    message(STATUS "fitpackpp could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        fitpackpp_external
        GIT_REPOSITORY https://github.com/LarsHaalck/fitpackpp
        GIT_SHALLOW ON
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DBUILD_EXAMPLES=OFF
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(fitpackpp_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/fitpackpp)
endif()
