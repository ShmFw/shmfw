include_directories(./include)
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

add_executable(shmfw-vtk src/main_vtk.cpp )
target_link_libraries(shmfw-vtk ${Boost_LIBRARIES} rt vtkHybrid)
install (TARGETS shmfw-vtk DESTINATION $ENV{HOME}/bin)
