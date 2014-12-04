IF(EIGEN_FOUND)

add_executable(shmfw-example_eigen_shmfw src/example_eigen_shmfw.cpp )
target_link_libraries(shmfw-example_eigen_shmfw)

ENDIF (EIGEN_FOUND)