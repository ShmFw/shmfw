add_library(gazebo_shm_laser SHARED gazebo_shm_laser.cc)
target_link_libraries(gazebo_shm_laser ${GAZEBO_LIBRARIES} RayPlugin rt ${Boost_LIBRARIES})
