add_executable(shmfw-example_usage main_usage.cpp )
target_link_libraries(shmfw-example_usage ${Boost_LIBRARIES} rt shmfw)

add_executable(shmfw-example_usage_raw main_usage_raw.cpp )
target_link_libraries(shmfw-example_usage_raw ${Boost_LIBRARIES} rt shmfw)
