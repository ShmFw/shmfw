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

install(FILES ${FILES_H} DESTINATION "${INSTALL_INCLUDE_DIR}/shmfw/objects")


