
add_executable(shmfw-reconfigure_example reconfigure_example.cpp )
target_link_libraries(shmfw-reconfigure_example shmfw ${Boost_LIBRARIES} rt)
