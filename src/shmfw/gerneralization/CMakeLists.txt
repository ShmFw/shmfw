
SET(FILES_H
  handler_allocator.h
  handler_deque.h
  handler_object.h
  handler_variable.h
  handler_vector.h
)
SET(FILES_CPP
  handler_object.cpp
)

add_library(shmfw_gerneralization SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(shmfw_gerneralization shmfw_objects)
IF(MRPT_FOUND)
target_link_libraries(shmfw_gerneralization ${MRPT_LIBS})
ENDIF(MRPT_FOUND)

_install_library(shmfw_gerneralization "${FILES_H}" "/shmfw/gerneralization")
