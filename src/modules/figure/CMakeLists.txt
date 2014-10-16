FIND_PACKAGE(OpenCV)
IF(OpenCV_FOUND)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(./include )

add_library(shmfw_figure SHARED src/figure.cpp)
target_link_libraries(shmfw_figure  ${Boost_LIBRARIES} rt ${OpenCV_LIBS})

add_executable(shmfw-figure src/main_figure.cpp)
target_link_libraries(shmfw-figure  shmfw_figure ${Boost_LIBRARIES} rt ${OpenCV_LIBS})
install (TARGETS shmfw-figure DESTINATION $ENV{HOME}/bin)
install (DIRECTORY "include/shmfw"  DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(TARGETS shmfw_figure DESTINATION ${LIBRARY_INSTALL_PATH} )

ENDIF(OpenCV_FOUND)