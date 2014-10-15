/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <Eigen/Dense>
#include <shmfw/variable.h>


int main() {
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( ShmFw::DEFAULT_SEGMENT_NAME(), ShmFw::DEFAULT_SEGMENT_SIZE() );
    ShmFw::Var<Eigen::Vector3d> a ( "Eigen::Vector3d",shmHdl );
    a() = Eigen::Vector3d::Random();
    ShmFw::Var<Eigen::Matrix3d> M ( "Eigen::Matrix3d",shmHdl );
    M() = Eigen::Matrix3d::Random();
    ShmFw::Var<Eigen::Vector3d> b ( "Eigen::Vector3d",shmHdl );
}
