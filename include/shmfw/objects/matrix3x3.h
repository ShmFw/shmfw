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

#ifndef SHARED_MEM_MATRIX3x3
#define SHARED_MEM_MATRIX3x3

#include <shmfw/objects/vector2.h>
#include <shmfw/objects/vector3.h>

namespace ShmFw {


template<typename T = float>
class Matrix3x3 {
public:
    T m00, m01, m02;
    T m10, m11, m12;
    T m20, m21, m22;
public:
    Matrix3x3 () {}
    Matrix3x3 ( const Matrix3x3 &v )
        : m00 ( v.m00 ), m01 ( v.m01 ), m02 ( v.m02 ), m10 ( v.m10 ), m11 ( v.m11 ), m12 ( v.m12 ), m20 ( v.m20 ), m21 ( v.m21 ), m22 ( v.m22 ) {}
    Matrix3x3 ( const T &m00, const T &m01, const T &m02, const T &m10, const T &m11, const T &m12, const T &m20, const T &m21, const T &m22 )
        : m00 ( m00 ), m01 ( m01 ), m02 ( m02 ), m10 ( m10 ), m11 ( m11 ), m12 ( m12 ), m20 ( m20 ), m21 ( m21 ), m22 ( m22 ) {}
    void clear() {
        m00 = 0, m01 = 0, m02 = 0;
        m10 = 0, m11 = 0, m12 = 0;
        m20 = 0, m21 = 0, m22 = 0;
    }
    void eye() {
        m00 = 1, m01 = 0, m02 = 0;
        m10 = 0, m11 = 1, m12 = 0;
        m20 = 0, m21 = 0, m22 = 1;
    }
    void rand ( const T& min, const T& max ) {
        T d = max - min;
        m00 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m01 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m02 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m10 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m11 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m12 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m20 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m21 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        m22 = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
    }
    void setValues ( const T &_m00, const T &_m01, const T &_m02,
                     const T &_m10, const T &_m11, const T &_m12,
                     const T &_m20, const T &_m21, const T &_m22 ) {
        m00 = _m00, m01 = _m01, m02 = _m02;
        m10 = _m10, m11 = _m11, m12 = _m12;
        m20 = _m20, m21 = _m21, m22 = _m22;
    }
    T sum() {
        return m00 + m01 + m02 + m10 + m11 + m12 + m20 + m21 + m22;
    }
    Vector3<T>& normalize() {
        return ( *this ) /= sum();
    }
    T& operator() ( int row, int col ) {
        return ( &m00 ) [row*3+col];
    }
    const T& operator() ( int row, int col ) const {
        return ( &m00 ) [row*3+col];
    }
    Matrix3x3<T> operator = ( const Matrix3x3<T>& v ) {
        m00 = v.m00, m01 = v.m01, m02 = v.m02;
        m10 = v.m10, m11 = v.m11, m12 = v.m12;
        m20 = v.m20, m21 = v.m21, m22 = v.m22;
        return *this;
    }
    bool operator == ( const Matrix3x3<T>& v ) const {
        return  m00 == v.m00 && m01 == v.m01 && m02 == v.m02 && m10 == v.m10 && m11 == v.m11 && m12 == v.m12 && m20 == v.m20 && m21 == v.m21 && m22 == v.m22;
    }
    bool operator != ( const Vector3<T>& v ) const {
        return  m00 != v.m00 || m01 != v.m01 || m02 != v.m02 || m10 != v.m10 || m11 != v.m11 || m12 != v.m12 || m20 != v.m20 || m21 != v.m21 || m22 != v.m22;
    }
    Matrix3x3<T>& operator+= ( const Matrix3x3<T>& v ) {
        m00 += v.m00, m01 += v.m01, m02 += v.m02;
        m10 += v.m10, m11 += v.m11, m02 += v.m12;
        m20 += v.m00, m21 += v.m21, m22 += v.m22;
        return  *this;
    }
    Matrix3x3<T> operator+ ( const Matrix3x3<T>& v ) const {
        return  Matrix3x3<T> ( *this ) += v;
    }
    Matrix3x3<T>& operator-= ( const Matrix3x3<T>& v ) {
        m00 -= v.m00, m01 -= v.m01, m02 -= v.m02;
        m10 -= v.m10, m11 -= v.m11, m02 -= v.m12;
        m20 -= v.m00, m21 -= v.m21, m22 -= v.m22;
        return  *this;
    }
    Matrix3x3<T> operator- ( const Matrix3x3<T>& v ) const {
        return  Matrix3x3<T> ( *this ) -= v;
    }
    Matrix3x3<T>& operator*= ( const T& v ) {
        m00 *= v, m01 *= v, m02 *= v;
        m10 *= v, m11 *= v, m02 *= v;
        m20 *= v, m21 *= v, m22 *= v;
        return  *this;
    }
    Matrix3x3<T>& operator/= ( const T& v ) {
        m00 /= v, m01 /= v, m02 /= v;
        m10 /= v, m11 /= v, m02 /= v;
        m20 /= v, m21 /= v, m22 /= v;
        return *this;
    }
    Matrix3x3<T> operator/ ( const T& v ) const {
        return Matrix3x3<T> ( *this ) /= v;
    }
    const Matrix3x3<T>& operator* ( const T& v ) const {
        return  Matrix3x3<T> ( *this ) *= v;
    }
    Vector3<T> operator* ( const Vector3<T>& v ) const {
        return Vector3<T> ( m00 * v.x + m01 * v.y + m02 * v.z,
                            m10 * v.x + m11 * v.y + m12 * v.z,
                            m20 * v.x + m21 * v.y + m22 * v.z );
    }
    Matrix3x3<T> operator* ( const Matrix3x3<T>& v ) const {
        return Matrix3x3<T> ( m00 * v.m00 + m01 * v.m10 + m02 * v.m20,
                              m00 * v.m01 + m01 * v.m11 + m02 * v.m21,
                              m00 * v.m02 + m01 * v.m12 + m02 * v.m22,
                              m10 * v.m00 + m11 * v.m10 + m12 * v.m20,
                              m10 * v.m01 + m11 * v.m11 + m12 * v.m21,
                              m10 * v.m02 + m11 * v.m12 + m12 * v.m22,
                              m20 * v.m00 + m21 * v.m10 + m22 * v.m20,
                              m20 * v.m01 + m21 * v.m11 + m22 * v.m21,
                              m20 * v.m02 + m21 * v.m12 + m22 * v.m22 );
    }
    Matrix3x3<T> transpose() const  {
        return Matrix3x3<T> ( m00, m10, m20, m01, m11, m21, m02, m12, m22 );
    }
    Matrix3x3<T>& operator*= ( const Matrix3x3<T>& v ) {
        *this = ( *this ) * v;
        return *this;
    }
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << std::setw ( 12 ) << m00 <<  ", " << std::setw ( 12 ) << m01 <<  ", " << std::setw ( 12 ) << m02 << "; " << std::endl;
        ss << "  " << std::setw ( 12 ) << m10 <<  ", " << std::setw ( 12 ) << m11 <<  ", " << std::setw ( 12 ) << m12 << "; " << std::endl;
        ss << "  " << std::setw ( 12 ) << m20 <<  ", " << std::setw ( 12 ) << m21 <<  ", " << std::setw ( 12 ) << m22 << "]";
        return ss.str();
    }

    T det() const {
        return m00 * ( m11 * m22 - m21 * m12 ) + m10 * ( m21 * m02 - m01 * m22 ) + m20 * ( m01 * m12 - m11 * m02 );
    }
    static T det2 ( T a, T b, T c, T d ) {
        return a * d - b * c;
    }
    /// ToDo there is problem
    Matrix3x3<T> adjoint() const {
        return Matrix3x3<T> ( det2 ( m11, m12, m21, m22 ),
                              det2 ( m12, m10, m22, m20 ),
                              det2 ( m10, m11, m20, m21 ),
                              det2 ( m02, m01, m22, m21 ),
                              det2 ( m00, m02, m20, m22 ),
                              det2 ( m01, m00, m21, m20 ),
                              det2 ( m01, m02, m11, m12 ),
                              det2 ( m02, m00, m12, m10 ),
                              det2 ( m00, m01, m10, m11 ) );
    }
    Matrix3x3<T> invert() const {
        return adjoint().transpose() / det();
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::Matrix3x3<T> &o ) {
        os << "[ " << o.m00 <<  ", " << o.m01 <<  ", " << o.m02 << "; ";
        os         << o.m10 <<  ", " << o.m11 <<  ", " << o.m12 << "; ";
        os         << o.m21 <<  ", " << o.m21 <<  ", " << o.m22 << "]";
        return os;
    };
    /**
     * sets the Matrix3x3 from a quaterion
     * @param x x
     * @param y y
     * @param z x
     * @param w x
     **/
    void setRotation ( T x, T y, T z, T w ) {
        T  d = x*x+y*y+z*z+w*w;
        if ( d != 0.0 ) {
            T s = T ( 2.0 ) / d;
            T xs = x * s,   ys = y * s,   zs = z * s;
            T wx = w * xs,  wy = w * ys,  wz = w * zs;
            T xx = x * xs,  xy = x * ys,  xz = x * zs;
            T yy = y * ys,  yz = y * zs,  zz = z * zs;
            this->setValues ( T ( 1.0 ) - ( yy + zz ), xy - wz,                 xz + wy,
                              xy + wz,                 T ( 1.0 ) - ( xx + zz ), yz - wx,
                              xz - wy,                 yz + wx,                 T ( 1.0 ) - ( xx + yy ) );
        }
    }
protected:
    friend class boost::serialization::access;
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "m00", m00 );
        ar & make_nvp ( "m01", m01 );
        ar & make_nvp ( "m02", m02 );
        ar & make_nvp ( "m10", m10 );
        ar & make_nvp ( "m11", m11 );
        ar & make_nvp ( "m12", m12 );
        ar & make_nvp ( "m20", m20 );
        ar & make_nvp ( "m21", m21 );
        ar & make_nvp ( "m22", m22 );
    }
};
};


#endif //SHARED_MEM_MATRIX3x3

