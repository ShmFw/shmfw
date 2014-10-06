
add_library(shmfw-pose2d_agv STATIC pose2d_agv.cpp )
target_link_libraries(shmfw-pose2d_agv  ${Boost_LIBRARIES} rt)
SET_TARGET_PROPERTIES( shmfw-pose2d_agv PROPERTIES COMPILE_FLAGS -fPIC)