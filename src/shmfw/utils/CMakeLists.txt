SET(FILES_H
  matlab.h
)
SET(FILES_CPP
  matlab.cpp
)


install(FILES ${FILES_H} DESTINATION "${INSTALL_INCLUDE_DIR}/shmfw/serialization")
add_library(shmfw_utils SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(shmfw_utils)