SET(APPS_VIEWER 1 CACHE BOOL "shmfw viewer" )
IF(APPS_VIEWER )

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-view_laser_scan view_laser_scan.cpp )
target_link_libraries(shmfw-view_laser_scan  shmfw_core  shmfw_objects ${CURSES_LIBRARY})
_install_binary (shmfw-view_laser_scan)

add_executable(shmfw-view_twist view_twist.cpp )
target_link_libraries(shmfw-view_twist  shmfw_core  shmfw_objects ${CURSES_LIBRARY})
_install_binary (shmfw-view_twist)


ENDIF(APPS_VIEWER )

 