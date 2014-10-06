include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

IF(GAZEBO_FOUND)

include_directories(plugins/include)
add_subdirectory(plugins)
add_subdirectory(examples)

ENDIF(GAZEBO_FOUND)
