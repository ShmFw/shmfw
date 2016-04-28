SET(APPS_IMAGE 1 CACHE BOOL "shmfw image tools" )
IF(APPS_IMAGE AND  OpenCV_FOUND ) 

add_executable(shmfw-image_view image_view.cpp )
target_link_libraries(shmfw-image_view  shmfw_core shmfw_objects ${OpenCV_LIBS})
_install_binary (shmfw-image_view)


ENDIF()