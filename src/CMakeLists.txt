
SET(VTK_ON 0 CACHE BOOL "vtk support" )


add_subdirectory(examples)
add_subdirectory(modules)
add_subdirectory(apps)
add_subdirectory(dynamic)

SET(SANDBOX 1 CACHE BOOL "shmfw sandbox" )
IF(SANDBOX )
add_subdirectory(sandbox)
ENDIF(SANDBOX )


SET(UNITTEST 0 CACHE BOOL "shmfw unittests" )
IF(UNITTEST )
add_subdirectory(unit_tests)
ENDIF(UNITTEST )



SET(SHMFW_DYNAMIC_LIBRARIES
  shmfw-pose2d_agv
)

add_library(shmfw SHARED shmfw.cpp)
target_link_libraries(shmfw ${Boost_LIBRARIES} rt  ${SHMFW_DYNAMIC_LIBRARIES})
IF(MRPT_FOUND)
target_link_libraries(shmfw ${MRPT_LIBS})
ENDIF(MRPT_FOUND)

install(TARGETS shmfw DESTINATION ${LIBRARY_INSTALL_PATH} )
