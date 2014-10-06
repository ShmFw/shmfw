FIND_PACKAGE( OpenCV)
IF( OpenCV_FOUND ) 

add_executable(shmfw-example_image-serialize main_image_serialize.cpp )
target_link_libraries(shmfw-example_image-serialize  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})

ENDIF()
 