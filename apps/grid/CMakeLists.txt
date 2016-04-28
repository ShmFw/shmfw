SET(APPS_GRID 1 CACHE BOOL "shmfw grid viewer and tools" )
IF(APPS_GRID )

IF( OpenCV_FOUND ) 
add_executable(shmfw-grid_view_as_image grid_view_as_image.cpp )
target_link_libraries(shmfw-grid_view_as_image  shmfw_core  shmfw_objects ${OpenCV_LIBS})
_install_binary (shmfw-grid_view_as_image)

add_executable(shmfw-grid_view_as_plot grid_view_as_plot.cpp )
target_link_libraries(shmfw-grid_view_as_plot  shmfw_core  shmfw_objects  ${OpenCV_LIBS})
_install_binary (shmfw-grid_view_as_plot)

ENDIF(OpenCV_FOUND)

IF(MATHGL_FOUND)

add_executable(shmfw-grid_view grid_view.cpp)
target_link_libraries(shmfw-grid_view shmfw_objects shmfw_core -pthread ${MATHGL_LIBRARIES})
_install_binary (shmfw-grid_view)

add_executable(shmfw-grid_view2 grid_view2.cpp)
target_link_libraries(shmfw-grid_view2 shmfw_objects shmfw_core -pthread ${MATHGL_LIBRARIES})
#_install_binary (shmfw-grid_view2)

endif(MATHGL_FOUND)
endif(APPS_GRID)