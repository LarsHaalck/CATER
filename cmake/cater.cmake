set(targets_export_name caterTargets)
set(cater_cmake_dir "${CMAKE_INSTALL_LIBDIR}/cmake/cater")
set_property(GLOBAL PROPERTY cater_comps "")

function(install_logic)
  set(version_file "${PROJECT_BINARY_DIR}/cater-config-version.cmake")
  set(config_file "${PROJECT_BINARY_DIR}/cater-config.cmake")

  include(GNUInstallDirs)
  install(
    EXPORT ${targets_export_name}
    FILE ${targets_export_name}.cmake
    NAMESPACE cater::
    DESTINATION ${cater_cmake_dir}
  )


  include(CMakePackageConfigHelpers)
  get_property(comps GLOBAL PROPERTY cater_comps)
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cater-config.cmake.in
    ${config_file}
    INSTALL_DESTINATION ${cater_cmake_dir}
  )
  write_basic_package_version_file(
    ${version_file}
    COMPATIBILITY SameMajorVersion
  )
  install(
    FILES ${config_file} ${version_file}
    DESTINATION ${cater_cmake_dir}
  )


endfunction()

function(install_lib lib with_includes)
  install(
    TARGETS ${lib}
    EXPORT ${targets_export_name}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

  if(${with_includes})
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION .)
  endif()
endfunction()

function(install_bin bin)
  install(
    TARGETS ${bin}
    DESTINATION bin
  )
endfunction()

function(install_config lib)
  set_property(GLOBAL APPEND PROPERTY cater_comps ${lib})
  configure_file(${lib}-config.cmake.in ${lib}-config.cmake)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${lib}-config.cmake
    DESTINATION ${cater_cmake_dir}
  )
endfunction()
