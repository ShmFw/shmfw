
SET(MODULE_COM 0 CACHE BOOL "com modul" )
IF(MODULE_COM )
add_subdirectory(com)
ENDIF(MODULE_COM )

SET(MODULE_V4LCAM 0 CACHE BOOL "v4l modul" )
IF(MODULE_V4LCAM )
add_subdirectory(v4lcam)
ENDIF(MODULE_V4LCAM )


SET(MODULE_OPENCV_CAMERA 1 CACHE BOOL "opencv camera modul" )
IF(MODULE_OPENCV_CAMERA )
add_subdirectory(opencv_camera)
ENDIF(MODULE_OPENCV_CAMERA )


SET(MODULE_OPENCV_FIGURE 0 CACHE BOOL "opencv figure modul" )
IF(MODULE_OPENCV_FIGURE )
add_subdirectory(figure)
ENDIF(MODULE_OPENCV_FIGURE )

SET(MODULE_GAZEBOSIM 0 CACHE BOOL "gazebo sim plugins" )
IF(MODULE_GAZEBOSIM )
add_subdirectory(gazebosim)
ENDIF(MODULE_GAZEBOSIM )
