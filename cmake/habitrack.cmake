set(targets_export_name habitrackTargets)

function(install_logic)
    set(version_file "${PROJECT_BINARY_DIR}/habitrack-config-version.cmake")
    set(config_file "${PROJECT_BINARY_DIR}/habitrack-config.cmake")
    set(habitrack_cmake_dir "${CMAKE_INSTALL_LIBDIR}/cmake/habitrack")

    include(GNUInstallDirs)
    install(EXPORT ${targets_export_name}
        FILE ${targets_export_name}.cmake
        NAMESPACE habitrack::
        DESTINATION ${habitrack_cmake_dir}
    )


    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/habitrack-config.cmake.in
        ${config_file}
        INSTALL_DESTINATION ${habitrack_cmake_dir}
    )
    write_basic_package_version_file(
        ${version_file}
        COMPATIBILITY SameMajorVersion
    )
    install(
        FILES ${config_file} ${version_file}
        DESTINATION ${habitrack_cmake_dir})
endfunction()

function(install_lib lib with_includes)
    install(TARGETS ${lib}
        EXPORT ${targets_export_name}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    if (${with_includes})
        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION .)
    endif()
endfunction()
