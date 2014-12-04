FIND_PACKAGE( OpenCV)
IF( OpenCV_FOUND ) 

add_executable(shmfw-example_image main_image.cpp )
target_link_libraries(shmfw-example_image  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})


add_executable(shmfw-example_image_grid main_image_grid.cpp )
target_link_libraries(shmfw-example_image_grid  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})

ENDIF()
 