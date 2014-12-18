add_executable(usage_header usage_header.cpp )
target_link_libraries(usage_header )

add_executable(usage_var usage_var.cpp )
target_link_libraries(usage_var )

add_executable(usage_vector usage_vector.cpp )
target_link_libraries(usage_vector )

add_executable(usage_locking usage_locking.cpp )
target_link_libraries(usage_locking )

add_executable(usage_allocator usage_allocator.cpp )
target_link_libraries(usage_allocator )

add_executable(usage_dynamic_grid usage_dynamic_grid.cpp )
target_link_libraries(usage_dynamic_grid ${PREFIX_LIB}core ${PREFIX_LIB}objects)

IF(EIGEN_FOUND)

add_executable(usage_eigen usage_eigen.cpp )
target_link_libraries(usage_eigen)

ENDIF (EIGEN_FOUND)