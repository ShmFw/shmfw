/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <shmfw/serialization/mrpt.h>
#include <shmfw/serialization/io_file.h>
#include <time.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

int main() {


    srand ( time ( NULL ) );

    std::string filename ( "/tmp/variable_mrpt.xml" );
    std::cout << "-------------- Variable read/write --------------" << std::endl;
    std::cout << "to -> " << filename << std::endl;
    
    // The landmark (global) position: 3D (x,y,z)
    CPoint2D a ( rand(),rand());
    CPoint2D b;

    ShmFw::write ( filename, a, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, b, ShmFw::FORMAT_XML );

    std::cout  << a << " = " << b << std::endl;
}

