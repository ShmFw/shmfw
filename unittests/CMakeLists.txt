
SET(UNITTEST 0 CACHE BOOL "shmfw unittests" )
IF(UNITTEST )

include_directories(${GTEST_INCLUDE_DIR})
enable_testing()

# Add test cpp file
add_executable(unittest_shm unittest_shm.cpp)
target_link_libraries(unittest_shm gtest shmfw_core)

add_test(
    NAME unittest
    COMMAND unittest
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
  unittest_objects_transform2d.cpp
  unittest_objects_quaterion.cpp
  unittest_objects_marker.cpp
  unittest_objects_line2d.cpp
  unittest_objects_linesegment2d.cpp
  unittest_objects_grid_map.cpp)

  
add_executable(unittest_objects ${UNITTEST_OBJECTS_SOURCES})


target_link_libraries(unittest_objects gtest shmfw_core shmfw_objects)

add_test(
    NAME unittest_objects
    COMMAND unittest_objects
)

add_executable(unittest_eigen unittest_eigen.cpp)
target_link_libraries(unittest_eigen gtest shmfw_core shmfw_objects)

add_test(
    NAME unittest_eigen
    COMMAND unittest_eigen
)

IF(MRPT_FOUND)

add_executable(unittest_mrpt unittest_mrpt.cpp)
target_link_libraries(unittest_mrpt gtest shmfw_core shmfw_objects ${MRPT_LIBS})

add_test(
    NAME unittest_mrpt
    COMMAND unittest_mrpt
)

ENDIF(MRPT_FOUND)

add_executable(unittest_opencv unittest_opencv.cpp)
target_link_libraries(unittest_opencv gtest shmfw_core shmfw_objects ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} rt)

add_test(
    NAME unittest_opencv
    COMMAND unittest_opencv
)

ENDIF(UNITTEST )