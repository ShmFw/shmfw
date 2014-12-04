SET(FILES_H
  deque.h
  eigen.h
  header.h
  interprocess_deque.hpp
  interprocess_vector.hpp
  io_file.h
  mrpt.h
  opencv.h
  variable.h
  vector.h
)

install(FILES ${FILES_H} DESTINATION "${INSTALL_INCLUDE_DIR}/shmfw/serialization")

