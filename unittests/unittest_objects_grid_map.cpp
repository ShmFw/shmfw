#include <iostream>
#include <fstream>
#include "unittest_objects.h"
#include "shmfw/objects/dynamic_grid_map.h"
namespace ShmFwTest {


TEST_F ( ObjectTest, gridMapToMatlab ) {

    ShmFw::DynamicGridMap64FHeap grid;
    grid.setSizeWithResolution ( -10, 10, -10, 10, 1., 1., 1 );
    EXPECT_EQ(grid.getLayers(), 1); 
    EXPECT_EQ(grid.getResolutionX(), 1); 
    EXPECT_EQ(grid.getResolutionY(), 1); 
    EXPECT_EQ(grid.getSizeX(), 20); 
    EXPECT_EQ(grid.getSizeY(), 20); 
    double d = 0.5;

    double x, y, m = grid.getSizeX(), n = grid.getSizeY();
    for ( int i=0; i<m; i++ ) {
        for ( int j=0; j<n; j++ ) {
            x = i/ ( n-1. );
            y = j/ ( m-1. );
            double v = d*sin ( 2*M_PI*x ) *sin ( 3*M_PI*y ) + ( 1.0-d ) *cos ( 3*M_PI*x*y );
            grid.setCellByIndex ( i, j, v );
        }
    }
    
    std::ofstream outfile("/tmp/gridmap.m", std::ofstream::out);
    grid.matlab(outfile, "grid");
    outfile.close();   
    
}


}  // namespace

