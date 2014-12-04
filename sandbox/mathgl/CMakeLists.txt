if(MATHGL_FOUND)

add_executable(sandbox-mathgl main_mathgl.cpp)
target_link_libraries(sandbox-mathgl ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox-mathgl_class main_mathgl_class.cpp)
target_link_libraries(sandbox-mathgl_class ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox-mathgl_vector_field_fow main_mathgl_vector_field_fow.cpp)
target_link_libraries(sandbox-mathgl_vector_field_fow ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox-mathgl_vector_field_traj main_mathgl_vector_field_traj.cpp)
target_link_libraries(sandbox-mathgl_vector_field_traj ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox-mathgl_mesh main_mathgl_mesh.cpp)
target_link_libraries(sandbox-mathgl_mesh ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})


endif(MATHGL_FOUND)
