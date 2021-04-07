# cereal does not export version correctly
find_package(cereal CONFIG QUIET)

if(TARGET cereal)
    get_property(loc TARGET cereal PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found cereal at ${loc}")
    add_library(cereal_external INTERFACE)
else()
    message(STATUS "cereal could not be located. Building from source...")
    include(ExternalProject)
    ExternalProject_Add(
        cereal_external
        GIT_REPOSITORY https://github.com/USCiLab/cereal
        GIT_TAG v1.3.0
        GIT_SHALLOW TRUE
        CMAKE_ARGS
          -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
          -DCMAKE_BUILD_TYPE=Release
          -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
          -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
          -DJUST_INSTALL_CEREAL=ON
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
