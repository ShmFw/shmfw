
add_subdirectory(gtest)

include_directories(${GTEST_INCLUDE_DIR})
enable_testing()

# Add test cpp file
add_executable(shmfw-unittest_shm unittest_shm.cpp)
target_link_libraries(shmfw-unittest_shm gtest ${Boost_LIBRARIES} rt shmfw)

add_test(
    NAME shmfw-unittest
    COMMAND shmfw-unittest
)

SET(UNITTEST_OBJECTS_SOURCES
  unittest_objects.cpp 
  unittest_objects_vector4.cpp 
  unittest_objects_vector3.cpp 
  unittest_objects_vector2.cpp
  unittest_objects_point.cpp 
  unittest_objects_vec.cpp
  unittest_objects_matrix3x3.cpp
  unittest_objects_plane.cpp
  unittest_objects_twist.cpp
  unittest_objects_quaterion.cpp
  unittest_objects_marker.cpp)

  
add_executable(shmfw-unittest_objects ${UNITTEST_OBJECTS_SOURCES})


target_link_libraries(shmfw-unittest_objects gtest ${Boost_LIBRARIES} rt shmfw)

add_test(
    NAME shmfw-unittest_objects
    COMMAND shmfw-unittest_objects
)

add_executable(shmfw-unittest_eigen unittest_eigen.cpp)
target_link_libraries(shmfw-unittest_eigen gtest ${Boost_LIBRARIES} rt shmfw)

add_test(
    NAME shmfw-unittest_eigen
    COMMAND shmfw-unittest_eigen
)

IF(USE_MRPT)

add_executable(shmfw-unittest_mrpt unittest_mrpt.cpp)
target_link_libraries(shmfw-unittest_mrpt gtest ${Boost_LIBRARIES} rt shmfw ${MRPT_LIBS})

add_test(
    NAME shmfw-unittest_mrpt
    COMMAND shmfw-unittest_mrpt
)

ENDIF(USE_MRPT)
