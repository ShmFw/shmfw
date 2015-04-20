/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2012 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 *********************************************************a******************/

#include <shmfw/objects/transform.h>
#include <shmfw/objects/quaternion.h>

ShmFw::Transform::Transform ()
    : basis ( Matrix3x3<double>::EYE() ), origin ( 0,0,0 ) {};

ShmFw::Transform::Transform ( const Transform &o ) : basis ( o.basis ), origin ( o.origin ) {};

ShmFw::Transform::Transform(const Matrix3x3<double> &_basis, const Vector3<double> _origin)
    : basis ( _basis ), origin ( _origin ) {};
    
ShmFw::Transform::Transform(const Quaternion &_basis, const Vector3<double> _origin)
    : basis ( _basis.x, _basis.y, _basis.z, _basis.w ), origin ( _origin ) {};
    
std::string ShmFw::Transform::getToString() const {
    char buf[0xFF];
    sprintf ( buf, "[ [%lf, %lf, %lf], [ %3.2lf, %3.2lf, %3.2lf; %3.2lf, %3.2lf, %3.2lf; %3.2lf, %3.2lf, %3.2lf] ]", origin.x, origin.y, origin.z,
              basis.m00, basis.m01, basis.m02, basis.m10, basis.m11, basis.m12, basis.m20, basis.m21, basis.m22 );
    return std::string ( buf );
}
bool ShmFw::Transform::setFromString ( const std::string &str ) {
    int start = str.find ( "[" );
    int end = str.find_last_of ( "]" );
    std::string data = str.substr ( start+1, end-1 );
    boost::erase_all ( data, " " );
    if ( sscanf ( data.c_str(), "[%lf,%lf,%lf],[%lf,%lf,%lf;%lf,%lf,%lf;%lf,%lf,%lf]", &origin.x, &origin.y, &origin.z,
                  &basis.m00, &basis.m01, &basis.m02, &basis.m10, &basis.m11, &basis.m12, &basis.m20, &basis.m21, &basis.m22 ) == EOF ) return false;
    return true;
}
