IF(MRPT_FOUND)

add_executable(shmfw-example_mrpt_transformaions src/example_transformaion.cpp )
target_link_libraries(shmfw-example_mrpt_transformaions ${MRPT_LIBS})

add_executable(shmfw-example_mrpt_shmfw src/example_mrpt_shmfw.cpp )
target_link_libraries(shmfw-example_mrpt_shmfw ${MRPT_LIBS})

add_executable(shmfw-example_mrpt_shmfw_vector src/example_mrpt_shmfw_vector.cpp )
target_link_libraries(shmfw-example_mrpt_shmfw_vector ${MRPT_LIBS})

add_executable(shmfw-example_mrpt_serialization src/example_mrpt_serialization.cpp )
target_link_libraries(shmfw-example_mrpt_serialization ${MRPT_LIBS})

ENDIF(MRPT_FOUND)