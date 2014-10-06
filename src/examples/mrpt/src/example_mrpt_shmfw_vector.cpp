/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/utils/CTicTac.h>
#include <shmfw/variable.h>
#include <shmfw/vector.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

int main() {
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( DEFAULT_SEGMENT_NAME, DEFAULT_SEGMENT_SIZE );
    ShmFw::Vector<CPoint3D> points("mrpt_points",shmHdl);
    ShmFw::Vector<CPoint3D> transformed("mrpt_points_transformed",shmHdl);
    ShmFw::Var<CPose3D> P("mrpt_pose",shmHdl);
    ShmFw::Var<CQuaternion<double> > Q("quat",shmHdl);
    
    try {

	// set does all the locking for your
        P.set(CPose3D( 0.5f,0.5f,1.5f ,DEG2RAD ( -90.0f ),DEG2RAD ( 0 ),DEG2RAD ( -90.0f ) ));
	CPose3D R;
	
	// set does all the locking for your
	P.get(R);
	
	// manual locking
	points.lock();
	transformed.lock();
	points().clear();
	for(int i = 0; i < 10; i++){
	  points.push_back(CPoint3D(1,0,i));
	  transformed.push_back( (points().back() - R) ) ;
	}
	transformed.unlock();
	points.unlock();
	

        return 0;
    } catch ( exception &e ) {
        cerr << "EXCEPCTION: " << e.what() << endl;
        return -1;
    } catch ( ... ) {
        cerr << "Untyped excepcion!!";
        return -1;
    }
}

