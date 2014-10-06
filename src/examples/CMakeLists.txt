

add_subdirectory(usage)
add_subdirectory(serialization)


SET(EXAMPLE_OPENCV 1 CACHE BOOL "shmfw opencv example" )
IF(EXAMPLE_OPENCV )

add_subdirectory(opencv)

ENDIF(EXAMPLE_OPENCV )
 
 
SET(EXAMPLE_MRPT 1 CACHE BOOL "shmfw mrpt example" )
IF(EXAMPLE_MRPT )
add_subdirectory(mrpt)
ENDIF(EXAMPLE_MRPT )