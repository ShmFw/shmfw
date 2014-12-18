IF(MRPT_FOUND)

add_executable(mrpt_transformaions mrpt_transformaion.cpp )
target_link_libraries(mrpt_transformaions ${MRPT_LIBS})

add_executable(mrpt_var mrpt_var.cpp )
target_link_libraries(mrpt_var ${MRPT_LIBS})

add_executable(mrpt_vector mrpt_vector.cpp )
target_link_libraries(mrpt_vector ${MRPT_LIBS})

add_executable(mrpt_serialization mrpt_serialization.cpp )
target_link_libraries(mrpt_serialization ${MRPT_LIBS})

ENDIF(MRPT_FOUND)