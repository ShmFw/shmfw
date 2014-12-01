
add_executable(shmfw-log_example log_example.cpp )
target_link_libraries(shmfw-log_example  ${Boost_LIBRARIES} rt)
#install (TARGETS shmfw_log_demo DESTINATION $ENV{HOME}/bin)
