FIND_PACKAGE(OpenCV)
IF(OpenCV_FOUND)

include_directories(./include)
add_executable(shmfw-image-view src/main_image_view.cpp )
target_link_libraries(shmfw-image-view  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-image-view DESTINATION $ENV{HOME}/bin)

ENDIF(OpenCV_FOUND)
