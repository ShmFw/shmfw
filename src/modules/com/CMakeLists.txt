

include_directories(./include)
  
add_library(shmfw_com src/udp.cpp)
target_link_libraries(shmfw_com  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})

add_executable(shmfw-com src/main_udp.cpp)
target_link_libraries(shmfw-com  shmfw_com ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-com DESTINATION $ENV{HOME}/bin)