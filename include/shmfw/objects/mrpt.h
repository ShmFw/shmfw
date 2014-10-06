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
 ***************************************************************************/

#ifndef SHARED_MEM_MRPT
#define SHARED_MEM_MRPT

#ifdef USE_MRPT
#include <shmfw/objects/point2d.h>
#include <shmfw/objects/point.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/pose.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/bits.h>

namespace mrpt {
namespace poses {
inline std::istream& operator>> ( std::istream &input, CPoint2D &o ) {
    std::string str;
    getline ( input, str );
    size_t start = str.find ( "(" );
    size_t end = str.find_last_of ( ")" );
    std::string data = str.substr ( start+1, end-1 );
    if ( sscanf ( data.c_str(), "%lf,%lf", &o.m_coords[0], &o.m_coords[1] ) == EOF ) {
        throw;
    }
    return input;
}
inline CPoint2D& operator<<( CPoint2D &des, const ShmFw::Point2D& src) {
    des.m_coords[0] = src.x, des.m_coords[1] = src.y;
    return des;
}
inline bool operator==(const CPoint2D &a, const ShmFw::Point2D &b) {
    return a.m_coords[0] == b.x && a.m_coords[1] == b.y;
}
inline bool operator==(const ShmFw::Point2D &a, const CPoint2D &b){
    return a.x == b.m_coords[0] && a.y == b.m_coords[1];
}

inline std::istream& operator>> ( std::istream &input, CPoint3D &o ) {
    std::string str;
    getline ( input, str );
    size_t start = str.find ( "(" );
    size_t end = str.find_last_of ( ")" );
    std::string data = str.substr ( start+1, end-1 );
    if ( sscanf ( data.c_str(), "%lf,%lf,%lf", &o.m_coords[0], &o.m_coords[1], &o.m_coords[2] ) == EOF ) {
        throw;
    }
    return input;
}

inline CPoint3D& operator<<( CPoint3D &des, const ShmFw::Point& src) {
    des.m_coords[0] = src.x,  des.m_coords[1] = src.y,  des.m_coords[2] = src.z;
    return des;
}
inline bool operator==(const CPoint3D &a, const ShmFw::Point &b) {
    return a.m_coords[0] == b.x && a.m_coords[1] == b.y && a.m_coords[2] == b.z;
}
inline bool operator==(const ShmFw::Point &a, const CPoint3D &b){
    return a.x == b.m_coords[0] && a.y == b.m_coords[1] && a.z == b.m_coords[2];
}

inline std::istream& operator>> ( std::istream &input, CPose2D &o ) {
    std::string str;
    getline ( input, str );
    size_t start = str.find ( "(" );
    size_t end = str.find_last_of ( ")" );
    std::string data = str.substr ( start+1, end-1 );
    double x, y, angle;
    size_t found_deg = str.find ( "deg" );
    if ( found_deg!=std::string::npos ) {
        if ( sscanf ( data.c_str(), "%lf,%lf,%lfdeg", &x, &y, &angle ) == EOF ) {
            throw;
        }
        o = CPose2D ( x, y, mrpt::utils::DEG2RAD ( angle ) );
    } else {
        if ( sscanf ( data.c_str(), "%lf,%lf,%lf", &x, &y, &angle ) == EOF ) {
            throw;
        }
        o = CPose2D ( x, y, angle );
    }
    return input;
}

inline CPose2D& operator<<( CPose2D &des, const ShmFw::Pose2D& src) {
    des.m_coords[0] = src.position.x,  des.m_coords[1] = src.position.y,  des.phi() = src.orientation;
    return des;
}
inline bool operator==(const CPose2D &a, const ShmFw::Pose2D &b) {
    return a.m_coords[0] == b.position.x && a.m_coords[1] == b.position.y && a.phi() == b.orientation;
}
inline bool operator==(const ShmFw::Pose2D &a, const CPose2D &b){
    return a.position.x == b.m_coords[0] && a.position.y == b.m_coords[1] && a.orientation == b.phi();
}

inline std::istream& operator>> ( std::istream &input, CPose3D &o ) {
    std::string str;
    getline ( input, str );
    std::string header ( "(x,y,z,yaw,pitch,roll)=" );
    size_t found_header = str.find ( header );
    if ( found_header!=std::string::npos ) {
        str.erase ( 0,header.length() );
    }
    size_t start = str.find ( "(" );
    size_t end = str.find_last_of ( ")" );
    std::string data = str.substr ( start+1, end-1 );
    double x, y, z, yaw, pitch, roll;
    size_t found_deg = str.find ( "deg" );
    if ( found_deg!=std::string::npos ) {
        if ( sscanf ( data.c_str(), "%lf,%lf,%lf,%lfdeg,%lfdeg,%lfdeg", &x, &y, &z, &yaw, &pitch, &roll ) == EOF ) {
            throw;
        }
        o.setFromValues ( x, y, z, mrpt::utils::DEG2RAD ( yaw ), mrpt::utils::DEG2RAD ( pitch ), mrpt::utils::DEG2RAD ( roll ) );
    } else {
        if ( sscanf ( data.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &yaw, &pitch, &roll ) == EOF ) {
            throw;
        }
        o.setFromValues ( x, y, z, yaw, pitch, roll );
    }
    return input;
}

inline CPose3D& operator<<( CPose3D &des, const ShmFw::Pose& src) {
    des.m_coords[0] = src.position.x,  des.m_coords[1] = src.position.y;
    mrpt::math::CQuaternion<double> q(src.orientation.w, src.orientation.x, src.orientation.y, src.orientation.z);
    mrpt::math::CMatrixDouble33 R;
    q.rotationMatrix(R);
    des.setRotationMatrix(R);
    return des;
}
inline bool operator==(const CPose3D &a, const ShmFw::Pose &b) {
    bool pose_eq = a.m_coords[0] == b.position.x && a.m_coords[1] == b.position.y;
    bool orientation_eq = true;
    return pose_eq && orientation_eq;
}
inline bool operator==(const ShmFw::Pose &a, const CPose3D &b){
    bool pose_eq = a.position.x == b.m_coords[0] && a.position.y == b.m_coords[1];
    bool orientation_eq = true;
    return pose_eq && orientation_eq;
}

};

namespace math {
template<typename T>
inline std::ostream& operator<< ( std::ostream &output, const CQuaternion<T> &o ) {
    char buf[0xFF];
    sprintf ( buf, "[%lf,%lf,%lf,%lf]", ( double ) o.x(), ( double ) o.y(), ( double ) o.z(), ( double ) o.r() );
    output << buf;
    return output;
}
template<typename T>
inline std::istream& operator>> ( std::istream &input, CQuaternion<T> &o ) {
    std::string str;
    getline ( input, str );
    size_t start = str.find ( "[" );
    size_t end = str.find_last_of ( "]" );
    std::string data = str.substr ( start+1, end-1 );
    double x, y, z, r;
    if ( sscanf ( data.c_str(), "%lf,%lf,%lf,%lf", &x, &y, &z, &r ) == EOF ) {
        throw;
    }
    o = CQuaternion<T> ( r, x, y, z );
    return input;
}
};
};
namespace ShmFw{
  
inline Point2D& operator<<( Point2D& des, const mrpt::poses::CPoint2D &src) {
    des.x = src.m_coords[0], des.y = src.m_coords[1];
    return des;
}
inline Point& operator<<( Point& des, const mrpt::poses::CPoint3D &src) {
    des.x = src.m_coords[0], des.y = src.m_coords[1], des.z = src.m_coords[2];
    return des;
}
inline Pose2D& operator<<( Pose2D &des, const mrpt::poses::CPose2D &src) {
    des.position.x = src.m_coords[0],  des.position.y = src.m_coords[1],  des.orientation = src.phi();    
    return des;
}
inline Pose& operator<<( Pose &des, const mrpt::poses::CPose3D &src) {
    des.position.x = src.m_coords[0],  des.position.y = src.m_coords[1],  des.position.z = src.m_coords[2];
    des.orientation.setEuler(src.roll(), src.pitch(), src.yaw());
    return des;
}
}
#endif //USE_MRPT
#endif //SHARED_MEM_MRPT

