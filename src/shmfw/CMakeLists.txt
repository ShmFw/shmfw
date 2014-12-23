
SET(FILES_H
  allocator.h
  deque.h
  forward_declarations.h
  handler.h
  header.h
  log.h
  shmfw.h
  variable.h
  vector.h
  string.h
)
SET(FILES_CPP
  shmfw.cpp
  handler.cpp
  header.cpp
  string.cpp
)

add_library(shmfw_core SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(shmfw_core)
IF(MRPT_FOUND)
target_link_libraries(shmfw_core ${MRPT_LIBS})
ENDIF(MRPT_FOUND)

_install_library(shmfw_core "${FILES_H}" "/shmfw")



add_subdirectory(objects)
add_subdirectory(gerneralization)
add_subdirectory(serialization)
add_subdirectory(modules)

add_subdirectory(utils)