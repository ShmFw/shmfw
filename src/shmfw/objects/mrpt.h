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

#include <shmfw/objects/point2d.h>
#include <shmfw/objects/point.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/pose.h>
#include <shmfw/objects/matrix3x3.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/bits.h>

namespace ShmFw {
mrpt::poses::CPoint2D& copy ( const ShmFw::Point2D& src, mrpt::poses::CPoint2D &des );
mrpt::poses::CPoint3D& copy ( const ShmFw::Point& src, mrpt::poses::CPoint3D &des );
mrpt::poses::CPose2D& copy ( const ShmFw::Pose2D& src, mrpt::poses::CPose2D &des );
mrpt::poses::CPose2D& copy ( const ShmFw::Pose& src, mrpt::poses::CPose2D &des );
};

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

inline mrpt::poses::CPoint2D& operator<< ( mrpt::poses::CPoint2D &des, const ShmFw::Point2D& src ) {
    des.m_coords[0] = src.x, des.m_coords[1] = src.y;
    return des;
}
inline bool operator== ( const mrpt::poses::CPoint2D &a, const ShmFw::Point2D &b ) {
    return a.m_coords[0] == b.x && a.m_coords[1] == b.y;
}
inline bool operator== ( const ShmFw::Point2D &a, const mrpt::poses::CPoint2D &b ) {
    return a.x == b.m_coords[0] && a.y == b.m_coords[1];
}

inline mrpt::poses::CPoint3D& operator<< ( mrpt::poses::CPoint3D &des, const ShmFw::Point& src ) {
    des.m_coords[0] = src.x,  des.m_coords[1] = src.y,  des.m_coords[2] = src.z;
    return des;
}
inline bool operator== ( const mrpt::poses::CPoint3D &a, const ShmFw::Point &b ) {
    return a.m_coords[0] == b.x && a.m_coords[1] == b.y && a.m_coords[2] == b.z;
}
inline bool operator== ( const ShmFw::Point &a, const mrpt::poses::CPoint3D &b ) {
    return a.x == b.m_coords[0] && a.y == b.m_coords[1] && a.z == b.m_coords[2];
}

inline mrpt::poses::CPose2D& operator<< ( mrpt::poses::CPose2D &des, const ShmFw::Pose2D& src ) {
    des.m_coords[0] = src.position.x,  des.m_coords[1] = src.position.y,  des.phi() = src.orientation;
    return des;
}

inline mrpt::poses::CPose2D& operator<< ( mrpt::poses::CPose2D &des, const ShmFw::Pose& src ) {
    des.m_coords[0] = src.position.x,  des.m_coords[1] = src.position.y,  des.phi() = src.orientation.getAngleYaw();
    return des;
}

template<typename T1, typename T2>
inline mrpt::math::CMatrixFixedNumeric<T1,3,3> & operator<< ( mrpt::math::CMatrixFixedNumeric<T1,3,3> &des, const ShmFw::Matrix3x3<T2>& src ) {
    des ( 0,0 ) = src.m00, des ( 0,1 ) = src.m01, des ( 0,2 ) = src.m02;
    des ( 1,0 ) = src.m10, des ( 1,1 ) = src.m11, des ( 1,2 ) = src.m12;
    des ( 2,0 ) = src.m20, des ( 2,1 ) = src.m21, des ( 2,2 ) = src.m22;
    return des;
}

inline bool operator== ( const mrpt::poses::CPose2D &a, const ShmFw::Pose2D &b ) {
    return a.m_coords[0] == b.position.x && a.m_coords[1] == b.position.y && a.phi() == b.orientation;
}
inline bool operator== ( const ShmFw::Pose2D &a, const mrpt::poses::CPose2D &b ) {
    return a.position.x == b.m_coords[0] && a.position.y == b.m_coords[1] && a.orientation == b.phi();
}


inline mrpt::poses::CPose3D& operator<< ( mrpt::poses::CPose3D &des, const ShmFw::Pose& src ) {
    des.m_coords[0] = src.position.x,  des.m_coords[1] = src.position.y,  des.m_coords[2] = src.position.z;
    mrpt::math::CQuaternion<double> q ( src.orientation.w, src.orientation.x, src.orientation.y, src.orientation.z );
    mrpt::math::CMatrixDouble33 R;
    q.rotationMatrix ( R );
    //ShmFw::Matrix3x3<double> M;
    //M.setRotation(src.orientation.x, src.orientation.y, src.orientation.z, src.orientation.w);
    //R(0,0) = M.m00, R(0,1) = M.m01, R(0,2) = M.m02, R(1,0) = M.m10, R(1,1) = M.m11, R(1,2) = M.m12, R(2,0) = M.m20, R(2,1) = M.m21, R(2,2) = M.m22;
    des.setRotationMatrix ( R );
    return des;
}
inline bool operator== ( const mrpt::poses::CPose3D &a, const ShmFw::Pose &b ) {
    bool pose_eq = a.m_coords[0] == b.position.x && a.m_coords[1] == b.position.y;
    bool orientation_eq = true;
    return pose_eq && orientation_eq;
}
inline bool operator== ( const ShmFw::Pose &a, const mrpt::poses::CPose3D &b ) {
    bool pose_eq = a.position.x == b.m_coords[0] && a.position.y == b.m_coords[1];
    bool orientation_eq = true;
    return pose_eq && orientation_eq;
}


template<typename T1, typename T2>
inline ShmFw::Matrix3x3<T1> & operator<< ( ShmFw::Matrix3x3<T1>& des, const mrpt::math::CMatrixFixedNumeric<T2,3,3> &src ) {
    des.m00 = src ( 0,0 ),  des.m01 = src ( 0,1 ), des.m02 = src ( 0,2 );
    des.m10 = src ( 1,0 ),  des.m11 = src ( 1,1 ), des.m12 = src ( 1,2 );
    des.m20 = src ( 2,0 ),  des.m21 = src ( 2,1 ), des.m22 = src ( 2,2 );
    return des;
}
inline ShmFw::Point2D& operator<< ( ShmFw::Point2D& des, const mrpt::poses::CPoint2D &src ) {
    des.x = src.m_coords[0], des.y = src.m_coords[1];
    return des;
}
inline ShmFw::Point& operator<< ( ShmFw::Point& des, const mrpt::poses::CPoint3D &src ) {
    des.x = src.m_coords[0], des.y = src.m_coords[1], des.z = src.m_coords[2];
    return des;
}
inline ShmFw::Pose2D& operator<< ( ShmFw::Pose2D &des, const mrpt::poses::CPose2D &src ) {
    des.position.x = src.m_coords[0],  des.position.y = src.m_coords[1],  des.orientation = src.phi();
    return des;
}
inline ShmFw::Pose& operator<< ( ShmFw::Pose &des, const mrpt::poses::CPose3D &src ) {
    des.position.x = src.m_coords[0],  des.position.y = src.m_coords[1],  des.position.z = src.m_coords[2];
    ShmFw::Matrix3x3<double> R;
    R << src.getRotationMatrix();
    R.getRotation ( des.orientation.x, des.orientation.y, des.orientation.z, des.orientation.w );
    return des;
}

#endif //SHARED_MEM_MRPT

