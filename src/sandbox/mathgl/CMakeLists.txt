if(USE_MATHGL)

add_executable(sandbox_mathgl main_mathgl.cpp)
target_link_libraries(sandbox_mathgl ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox_mathgl_class main_mathgl_class.cpp)
target_link_libraries(sandbox_mathgl_class ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox_mathgl_vector_field_fow main_mathgl_vector_field_fow.cpp)
target_link_libraries(sandbox_mathgl_vector_field_fow ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})

add_executable(sandbox_mathgl_vector_field_traj main_mathgl_vector_field_traj.cpp)
target_link_libraries(sandbox_mathgl_vector_field_traj ${Boost_LIBRARIES} -pthread rt ${MATHGL_LIBRARIES})
endif(USE_MATHGL)
