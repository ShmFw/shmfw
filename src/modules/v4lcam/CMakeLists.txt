
include_directories(./include)

SET(SHMFW_V4L2_SOURCES
  src/color.c 
  src/utils.c 
  src/v4l2uvc.c
  src/v4l_cam.cpp
  src/main.cpp)
  
add_executable(shmfw-v4l2 ${SHMFW_V4L2_SOURCES} )
target_link_libraries(shmfw-v4l2 m SDL v4l2 ${Boost_LIBRARIES} rt)
