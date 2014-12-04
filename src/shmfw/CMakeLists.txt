
SET(FILES_H
  allocator.h
  deque.h
  forward_declarations.h
  handler.h
  handler_allocator.h
  handler_deque.h
  handler_object.h
  handler_variable.h
  handler_vector.h
  header.h
  log.h
  shmfw.h
  variable.h
  vector.h
)
SET(FILES_CPP
  shmfw.cpp
)

add_library(${PREFIX_LIB}core SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(${PREFIX_LIB}core ${Boost_LIBRARIES} rt)
IF(MRPT_FOUND)
target_link_libraries(${PREFIX_LIB}core ${MRPT_LIBS})
ENDIF(MRPT_FOUND)

_install_library(${PREFIX_LIB}core "${FILES_H}" "/shmfw")

add_subdirectory(objects)
add_subdirectory(serialization)
#add_subdirectory(modules)
