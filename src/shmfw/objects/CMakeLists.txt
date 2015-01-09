add_subdirectory(ros)

SET(FILES_H
  agent_state.h
  base_object.h
  dynamic_grid_map.h
  eigen.h
  grid_map.h
  grid_map_header.h
  handler_parameterentry.h
  image.h
  line2d.h
  linesegment2d.h
  matrix3x3.h
  message.h
  model_state.h
  mrpt.h
  opencv.h
  parameterentry.h
  plane.h
  point.h
  point2d.h
  points.h
  pose.h
  pose2d.h
  pose2d_agv.h
  probability_entry.h
  quaternion.h
  rgb.h
  rgba.h
  route_segment.h
  twist.h
  vec.h
  vector2.h
  vector3.h
  vector4.h
)
SET(FILES_CPP
  mrpt.cpp
  grid_map_header.cpp
  pose2d_agv.cpp 
)

add_library(shmfw_objects SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(shmfw_objects ${MRPT_LIBS} ${OpenCV_LIBRARIES})


_install_library(shmfw_objects "${FILES_H}" "/shmfw/objects")


