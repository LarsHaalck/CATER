find_package(spdlog 1.10 CONFIG QUIET)

if(TARGET spdlog::spdlog)
    get_property(loc TARGET spdlog::spdlog PROPERTY LOCATION)
    message(STATUS "Found spdlog version ${PACKAGE_VERSION} at ${loc}")
    add_library(spdlog_external INTERFACE)
else()
    message(STATUS "spdlog could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        spdlog_external
        GIT_REPOSITORY https://github.com/gabime/spdlog
        GIT_TAG v1.10.0
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
    set(spdlog_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/spdlog)
endif()
