FIND_PACKAGE( OpenCV)
IF( OpenCV_FOUND ) 

include_directories(./include)
add_library(shmfw_oszi src/oszi.cpp)
target_link_libraries(shmfw_oszi ${PREFIX_LIB}core ${OpenCV_LIBS})

add_executable(shmfw-oszi src/main_oszi.cpp)
target_link_libraries(shmfw-oszi  shmfw_oszi ${PREFIX_LIB}core ${OpenCV_LIBS})
install (TARGETS shmfw-oszi DESTINATION bin)

add_subdirectory(examples)

ENDIF(OpenCV_FOUND )