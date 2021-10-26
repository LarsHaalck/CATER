###########################################################################
#                              DEPENDENCIES                               #
###########################################################################
find_package(Qt6Widgets 6 REQUIRED)
find_package(Qt6Concurrent 6 REQUIRED)
find_package(Qt6Multimedia 6 REQUIRED)

add_subdirectory(common)
add_subdirectory(habitrack)
add_subdirectory(panowizard)
