SET(APPS_VIEWER 1 CACHE BOOL "shmfw viewer" )
IF(APPS_VIEWER )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(${PREFIX_BIN}view_laser_scan view_laser_scan.cpp )
target_link_libraries(${PREFIX_BIN}view_laser_scan  ${PREFIX_LIB}core ${CURSES_LIBRARY})
_install_binary (${PREFIX_BIN}view_laser_scan)

add_executable(${PREFIX_BIN}view_twist view_twist.cpp )
target_link_libraries(${PREFIX_BIN}view_twist  ${PREFIX_LIB}core ${CURSES_LIBRARY})
_install_binary (${PREFIX_BIN}view_twist)

IF( OpenCV_FOUND ) 
add_executable(${PREFIX_BIN}view_image view_image.cpp )
target_link_libraries(${PREFIX_BIN}view_image  ${OpenCV_LIBS})
_install_binary (${PREFIX_BIN}view_image)

add_executable(${PREFIX_BIN}view_grid_as_image view_grid_as_image.cpp )
target_link_libraries(${PREFIX_BIN}view_grid_as_image  ${OpenCV_LIBS})
_install_binary (${PREFIX_BIN}view_grid_as_image)

add_executable(${PREFIX_BIN}view_grid_as_plot view_grid_as_plot.cpp )
target_link_libraries(${PREFIX_BIN}view_grid_as_plot  ${OpenCV_LIBS})
_install_binary (${PREFIX_BIN}view_grid_as_plot)

ENDIF(OpenCV_FOUND)


if(MATHGL_FOUND)

add_executable(${PREFIX_BIN}view_grid view_grid.cpp)
target_link_libraries(${PREFIX_BIN}view_grid -pthread ${MATHGL_LIBRARIES})
_install_binary (${PREFIX_BIN}view_grid)

add_executable(${PREFIX_BIN}view_grid2 view_grid2.cpp)
target_link_libraries(${PREFIX_BIN}view_grid2 -pthread ${MATHGL_LIBRARIES})
#_install_binary (${PREFIX_BIN}view_grid2)

endif(MATHGL_FOUND)


ENDIF(APPS_VIEWER )

 