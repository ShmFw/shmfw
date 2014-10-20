IF(USE_EIGEN)

add_executable(shmfw-example_eigen_shmfw src/example_eigen_shmfw.cpp )
target_link_libraries(shmfw-example_eigen_shmfw ${Boost_LIBRARIES} rt )

ENDIF (USE_EIGEN)