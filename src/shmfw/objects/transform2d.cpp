/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
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
 ***************************************************************************/

#include <shmfw/objects/transform2d.h>

ShmFw::Transform2D::Transform2D() {
};

ShmFw::Transform2D::Transform2D ( const Transform2D &o ) : M ( o.M ) {
};

ShmFw::Transform2D::Transform2D ( const double &dx, const double &dy, const double &da ) {
    setTf ( dx, dy, da );
};

ShmFw::Transform2D::Transform2D ( const ShmFw::Pose2D &o ) {
    setTf ( o );
};

ShmFw::Transform2D::Transform2D ( const Matrix3x3<double> &o ) : M ( o ) {
}

std::string ShmFw::Transform2D::getToStringFormat ( const std::string &format ) const {
    char buf[0xFF];
    sprintf ( buf, format.c_str(), x(), y(), orientation() );
    return std::string ( buf );
}

std::string ShmFw::Transform2D::getToString() const {
    return getToStringFormat ( "[ [%lf, %lf], [%lf] ]" );
}

bool ShmFw::Transform2D::setFromString ( const std::string &str ) {
    double dx, dy, da;
    int start = str.find ( "[" );
    int end = str.find_last_of ( "]" );
    std::string data = str.substr ( start+1, end-1 );
    boost::erase_all ( data, " " );
    if ( sscanf ( data.c_str(), "[%lf,%lf],[%lf]", &dx, &dy, &da ) == EOF ) {
        //  In the case of an input failure before any data could be successfully interpreted, EOF is returned.
        return false;  // Error
    }
    setTf ( dx, dy, da );
    return true;
}

bool ShmFw::Transform2D::operator == ( const Transform2D& o ) const {
    return M == o.M;
}

ShmFw::Point2D ShmFw::Transform2D::operator * ( const ShmFw::Point2D& o ) const {
    return ShmFw::Point2D ( o.x * M.m00 + o.y * M.m01 + M.m02, o.x * M.m10 + o.y * M.m11 + M.m12 );
}
const ShmFw::Transform2D &ShmFw::Transform2D::operator *= ( const ShmFw::Transform2D& o ) {
    M*=o.M;
    return *this;
}

ShmFw::Transform2D ShmFw::Transform2D::operator * ( const ShmFw::Transform2D& o ) const {
    ShmFw::Transform2D tf ( *this );
    return tf*=o;
}

bool ShmFw::Transform2D::equal ( const Transform2D& o, double tolerance ) const {
    double distance = fabs ( o.x() - x() ) + fabs ( o.y() - y() );
    double angle = o.orientation() - orientation();
    return ( distance < tolerance ) && ( angle < tolerance );
}

const double &ShmFw::Transform2D::x () const {
    return M.m02;
}

const double &ShmFw::Transform2D::y () const {
    return M.m12;
}

double ShmFw::Transform2D::orientation () const {
    return atan2 ( M.m10, M.m00 );
}

void ShmFw::Transform2D::getPose ( ShmFw::Pose2D &des ) const {
    des.setPose ( M.m02, M.m12, orientation () );
}

void ShmFw::Transform2D::setTf ( const ShmFw::Pose2D &src ) {
    setTf ( src.x(), src.y(), src.phi() );
}

void ShmFw::Transform2D::setTf ( const double &dx, const double &dy, const double &da ) {
    double ca = cos ( da ), sa = sin ( da );
    M.setValues ( ca, -sa, dx, sa, ca, dy, 0., 0., 1. );
}

void ShmFw::Transform2D::identity () {
    M.setValues ( 1., 0., 0., 0., 1., 0., 0., 0., 1. );
}

const ShmFw::Transform2D ShmFw::Transform2D::IDENTITY () {
    return ShmFw::Transform2D ( ShmFw::Matrix3x3<double>::EYE() );
}
