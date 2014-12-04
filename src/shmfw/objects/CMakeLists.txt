add_subdirectory(ros)

SET(FILES_H
  agent_state.h
  base_object.h
  dynamic_grid_map.h
  eigen.h
  grid_layer_map.h
  grid_map.h
  handler_parameterentry.h
  image.h
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
  grid_layer_map.cpp
  pose2d_agv.cpp 
)

add_library(${PREFIX_LIB}objects SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(${PREFIX_LIB}objects)
IF(MRPT_FOUND)
target_link_libraries(${PREFIX_LIB}objects ${MRPT_LIBS})
ENDIF(MRPT_FOUND)

_install_library(${PREFIX_LIB}objects "${FILES_H}" "/shmfw/objects")


