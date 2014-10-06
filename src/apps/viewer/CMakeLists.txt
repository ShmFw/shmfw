SET(APPS_VIEWER 1 CACHE BOOL "shmfw viewer" )
IF(APPS_VIEWER )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-view_laser_scan view_laser_scan.cpp )
target_link_libraries(shmfw-view_laser_scan  shmfw ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
install (TARGETS shmfw-view_laser_scan DESTINATION bin)

add_executable(shmfw-view_twist view_twist.cpp )
target_link_libraries(shmfw-view_twist  shmfw ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
install (TARGETS shmfw-view_twist DESTINATION bin)
ENDIF(APPS_VIEWER )
