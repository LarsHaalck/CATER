# if (NOT TARGET cereal)
#     message(WARNING "cxxopts not found, building from source....")
#     add_subdirectory(cereal)

#     install(TARGETS cereal
#         EXPORT habitrack-targets
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )
# endif()

# if (NOT TARGET ceres)
#     message(WARNING "cxxopts not found, building from source....")
#     add_subdirectory(ceres)

#     install(TARGETS ceres
#         EXPORT habitrack-targets
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )
# endif()

# if (NOT TARGET MILD)
#     message(WARNING "cxxopts not found, building from source....")
#     add_subdirectory(mild)

#     install(TARGETS mild
#         EXPORT habitrack-targets
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )
# endif()

# if (NOT TARGET opencv_core)
#     message(WARNING "cxxopts not found, building from source....")
#     add_subdirectory(opencv)

#     install(TARGETS mild
#         EXPORT habitrack-targets
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )
# endif()

# if (NOT TARGET cxxopts::cxxopts)
#     message(WARNING "cxxopts not found, building from source....")
#     add_subdirectory(cxxopts)
#     add_library(cxxopts::cxxopts ALIAS cxxopts)

#     install(TARGETS cxxopts
#         EXPORT habitrack-targets
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )
# endif()

# if (NOT TARGET spdlog::spdlog)
#     message(WARNING "spdlog not found, building from source....")
#     add_subdirectory(spdlog)
#     add_library(spdlog::spdlog ALIAS spdlog)

#     install(TARGETS spdlog
#         EXPORT habitrack-targets
#         DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     )
# endif()
