find_package(spdlog 1.8.5 CONFIG QUIET)

if(TARGET spdlog::spdlog)
    message(STATUS "Found spdlog version")
    add_library(spdlog_external INTERFACE) # dummy
else()
    message(STATUS "spdlog could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        spdlog_external
        GIT_REPOSITORY https://github.com/gabime/spdlog
        GIT_TAG v1.8.5
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
