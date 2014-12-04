FIND_PACKAGE(OpenCV)
IF(OpenCV_FOUND)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(./include )

set(FILES_H
  src/figure.h
)

set(FILES_CPP
  src/figure.cpp
)

add_library(${PREFIX_LIB}figure SHARED ${FILES_H} ${FILES_CPP})
target_link_libraries(${PREFIX_LIB}figure  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
_install_library(${PREFIX_LIB}figure ${FILES_CPP} ${FILES_H} "/v4r/geometry/3d")

add_executable(${PREFIX_BIN}figure src/main_figure.cpp)
target_link_libraries(${PREFIX_BIN}figure ${PREFIX_LIB}core ${PREFIX_LIB}figure ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
_install_binary(shmfw_figure)

ENDIF(OpenCV_FOUND)