add_executable(shmfw-example_usage_var main_usage_var.cpp )
target_link_libraries(shmfw-example_usage_var ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_and_tests main_usage_and_tests.cpp )
target_link_libraries(shmfw-example_usage_and_tests ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_raw main_usage_raw.cpp )
target_link_libraries(shmfw-example_usage_raw ${Boost_LIBRARIES} rt shmfw)

add_executable(shmfw-example_usage_allocator main_usage_allocator.cpp )
target_link_libraries(shmfw-example_usage_allocator ${Boost_LIBRARIES} rt shmfw)

