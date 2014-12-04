SET(FILES_H
  header.h
  laser_scan.h
  visualization_marker.h
)

install(FILES ${FILES_H} DESTINATION "${INSTALL_INCLUDE_DIR}/shmfw/objects/ros")
