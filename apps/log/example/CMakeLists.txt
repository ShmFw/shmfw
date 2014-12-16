
add_executable(shmfw-log_example log_example.cpp )
target_link_libraries(shmfw-log_example  ${PREFIX_LIB}core)
#install (TARGETS shmfw_log_demo DESTINATION $ENV{HOME}/bin)
