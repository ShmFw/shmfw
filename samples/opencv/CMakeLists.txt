FIND_PACKAGE( OpenCV)
IF( OpenCV_FOUND ) 

add_executable(opencv_image opencv_image.cpp )
target_link_libraries(opencv_image ${PREFIX_LIB}core  ${PREFIX_LIB}objects ${OpenCV_LIBS})


add_executable(opencv_image_grid opencv_image_grid.cpp )
target_link_libraries(opencv_image_grid ${PREFIX_LIB}core  ${PREFIX_LIB}objects ${OpenCV_LIBS})

ENDIF()
 