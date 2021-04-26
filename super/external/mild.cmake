find_package(mild CONFIG QUIET)

if(TARGET mild::mild)
    get_property(loc TARGET mild::mild PROPERTY LOCATION)
    message(STATUS "Found mild at ${loc}")
    add_library(mild_external INTERFACE)
else()
    message(STATUS "mild could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        mild_external
        DEPENDS
            eigen3_external
        GIT_REPOSITORY https://github.com/LarsHaalck/MILD/
        GIT_SHALLOW ON
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DBUILD_EXAMPLE=OFF
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(mild_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/mild)
endif()
