IF(VTK_ON)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

add_executable(vtk_sandbox vtk_sandbox.cpp )
target_link_libraries(vtk_sandbox vtkHybrid)

add_executable(vtk_sandbox_geometric_object_demo vtk_sandbox_geometric_object_demo.cpp )
target_link_libraries(vtk_sandbox_geometric_object_demo vtkHybrid)

add_executable(vtk_sandbox_procrustes_alignment_filter vtk_sandbox_procrustes_alignment_filter.cpp )
target_link_libraries(vtk_sandbox_procrustes_alignment_filter vtkHybrid)

add_executable(vtk_sandbox_cone4 vtk_sandbox_cone4.cpp )
target_link_libraries(vtk_sandbox_cone4 vtkHybrid)

ENDIF(VTK_ON )