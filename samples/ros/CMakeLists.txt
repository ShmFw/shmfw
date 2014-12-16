add_executable(shmfw-ros_visualization_marker main_visualization_marker.cpp )
target_link_libraries(shmfw-ros_visualization_marker ${PREFIX_LIB}core  ${PREFIX_LIB}objects)
