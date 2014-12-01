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

IF( OpenCV_FOUND ) 
add_executable(shmfw-view_image view_image.cpp )
target_link_libraries(shmfw-view_image  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-view_image DESTINATION bin)

add_executable(shmfw-view_grid_as_image view_grid_as_image.cpp )
target_link_libraries(shmfw-view_grid_as_image  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-view_grid_as_image DESTINATION bin)

add_executable(shmfw-view_grid_as_plot view_grid_as_plot.cpp )
target_link_libraries(shmfw-view_grid_as_plot  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-view_grid_as_plot DESTINATION bin)

ENDIF(OpenCV_FOUND)


if(MATHGL_FOUND)

add_executable(shmfw-view_grid view_grid.cpp)
target_link_libraries(shmfw-view_grid ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})
install (TARGETS shmfw-view_grid DESTINATION bin)

add_executable(shmfw-view_grid2 view_grid2.cpp)
target_link_libraries(shmfw-view_grid2 ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})
#install (TARGETS shmfw-view_grid2 DESTINATION bin)

endif(MATHGL_FOUND)


ENDIF(APPS_VIEWER )

 