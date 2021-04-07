find_package(OpenCV 4.2 CONFIG QUIET)

if(TARGET opencv_core)
    get_property(loc TARGET opencv_core PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found OpenCV version ${OpenCV_VERSION} at ${loc}")
    add_library(opencv_external INTERFACE) # dummy
else()
    message(STATUS "opencv could not be located.
        Building from source...
        It is strongly recommended to build OpenCV from source yourself due to
        many performance-improving dependencies. Check out build_opencv.sh")

    include(ExternalProject)
    ExternalProject_Add(
        opencv_external
        DEPENDS eigen3_external
        GIT_REPOSITORY https://github.com/opencv/opencv
        GIT_TAG 4.5.1
        GIT_SHALLOW TRUE
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${STAGED_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DEigen3_DIR=${Eigen3_DIR}
            -DBUILD_WITH_DEBUG_INFO=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_PERF_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DINSTALL_C_EXAMPLES=OFF
            -DINSTALL_PYTHON_EXAMPLES=OFF
            -DOPENCV_GENERATE_SETUPVARS=OFF
            -DOPENCV_ENABLE_NONFREE=OFF
        LOG_DOWNLOAD ON
        LOG_UPDATE ON
        LOG_PATCH ON
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
    set(OpenCV_DIR ${STAGED_INSTALL_PREFIX}/lib/cmake/opencv4)
    set(CMAKE_INSTALL_RPATH ${STAGED_INSTALL_PREFIX}/lib)
endif()
