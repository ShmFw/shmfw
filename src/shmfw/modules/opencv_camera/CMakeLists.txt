IF(OpenCV_FOUND)

add_executable(shmfw-opencv_camera opencv_camera.cpp)
target_link_libraries(shmfw-opencv_camera shmfw_core shmfw_objects ${OpenCV_LIBS})
install (TARGETS shmfw-opencv_camera DESTINATION $ENV{HOME}/bin)

ENDIF(OpenCV_FOUND)