add_executable(shmfw-example_serialization_variable main_serialization_variable.cpp )
target_link_libraries(shmfw-example_serialization_variable ${Boost_LIBRARIES} rt )

add_executable(shmfw-example_serialization_containter main_serialization_container.cpp )
target_link_libraries(shmfw-example_serialization_containter ${Boost_LIBRARIES} rt )

add_executable(shmfw-example_serialization_raw main_serialization_raw.cpp )
target_link_libraries(shmfw-example_serialization_raw ${Boost_LIBRARIES} rt shmfw)
