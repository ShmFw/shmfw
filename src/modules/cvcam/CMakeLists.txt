IF(USE_OpenCV)

add_executable(shmfw-cvcam src/main_cvcam.cpp)
target_link_libraries(shmfw-cvcam  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-cvcam DESTINATION $ENV{HOME}/bin)

ENDIF(USE_OpenCV)