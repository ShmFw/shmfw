

link_libraries(${PREFIX_LIB}core ${PREFIX_LIB}objects )

add_subdirectory(boost)
add_subdirectory(usage)
add_subdirectory(serialization)
add_subdirectory(ros)


SET(EXAMPLE_OPENCV 1 CACHE BOOL "shmfw opencv example" )
IF(EXAMPLE_OPENCV )

add_subdirectory(opencv)

ENDIF(EXAMPLE_OPENCV )
 
 
SET(EXAMPLE_MRPT 1 CACHE BOOL "shmfw mrpt example" )
IF(EXAMPLE_MRPT )
add_subdirectory(mrpt)
ENDIF(EXAMPLE_MRPT )