add_executable(shmfw-example_usage_header main_usage_header.cpp )
target_link_libraries(shmfw-example_usage_header ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_var main_usage_var.cpp )
target_link_libraries(shmfw-example_usage_var ${Boost_LIBRARIES} rt)


add_executable(shmfw-example_usage_vector main_usage_vector.cpp )
target_link_libraries(shmfw-example_usage_vector ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_dynamic_grid main_usage_dynamic_grid.cpp )
target_link_libraries(shmfw-example_usage_dynamic_grid ${Boost_LIBRARIES} rt)


add_executable(shmfw-example_usage_and_tests main_usage_and_tests.cpp )
target_link_libraries(shmfw-example_usage_and_tests ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_raw main_usage_raw.cpp )
target_link_libraries(shmfw-example_usage_raw ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_allocator main_usage_allocator.cpp )
target_link_libraries(shmfw-example_usage_allocator ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_container_of_container main_usage_container_of_container.cpp )
target_link_libraries(shmfw-example_usage_container_of_container ${Boost_LIBRARIES} rt)

add_executable(shmfw-example_usage_object_in_heap_or_shm main_usage_object_in_heap_or_shm.cpp )
target_link_libraries(shmfw-example_usage_object_in_heap_or_shm ${Boost_LIBRARIES} rt)


add_executable(shmfw-example_usage_copy_between_heap_and_shm main_usage_copy_between_heap_and_shm.cpp )
target_link_libraries(shmfw-example_usage_copy_between_heap_and_shm ${Boost_LIBRARIES} rt)