add_executable(shmfw-example_laser_scan main_laser_scan.cpp )
target_link_libraries(shmfw-example_laser_scan ${Boost_LIBRARIES} rt shmfw)