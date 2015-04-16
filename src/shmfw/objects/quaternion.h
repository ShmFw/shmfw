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
#ifndef SHARED_MEM_OBJECTS_QUATERNION_H
#define SHARED_MEM_OBJECTS_QUATERNION_H

#include <stdio.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/binary_object.hpp>
#include <shmfw/objects/vector3.h>
#include <shmfw/objects/vector4.h>
#include <shmfw/objects/base_object.h>
#include <boost/algorithm/string.hpp>

namespace ShmFw {
class Quaternion : public ShmFw::BaseObject {
public:
    double x, y, z, w;
    Quaternion() : x ( 0 ), y ( 0 ), z ( 0 ), w ( 1 ) {};
    Quaternion ( double x, double y, double z, double w ) : x ( x ), y ( y ), z ( z ), w ( w ) {};
    Quaternion ( const Quaternion &p ) : x ( p.x ), y ( p.y ), z ( p.z ), w ( p.w ) {};
    Quaternion ( const Vector3<double> axis, double angle ) {
        setRotation ( axis, angle );
    };
    std::string getToStringFormat ( const std::string &format ) const {
        char buf[0xFF];
        sprintf ( buf, format.c_str(), x, y, z, w );
        return std::string ( buf );
    }
    std::string getToString() const {
        return getToStringFormat ( "[ %lf, %lf, %lf, %lf]" );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%lf,%lf,%lf,%lf", &x, &y, &z, &w ) == EOF ) return false;
        return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const Quaternion &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Quaternion &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const Quaternion& o ) const {
        return x == o.x && y == o.y &&  z == o.z && w == o.w;
    }
    /**@brief Set the rotation using axis angle notation
    * @param axis The axis around which to rotate
    * @param angle The magnitude of the rotation in Radians */
    void setRotation ( const Vector3<double>& axis, double angle ) {
        double d = axis.norm();
        double s = sin ( angle *  0.5 ) / d;
        x = axis.x * s;
        y = axis.y * s;
        z = axis.z * s;
        w = cos ( angle * 0.5 );
    }
     /**
      * sets 0 0 0 1
     **/
    void zero(){
      x = 0, y = 0, z = 0, w = 1.;
    }
    void setValue ( double _x, double _y, double _z, double _w ) {
        this->x = _x, this->y = _y, this->z = _z, this->w = _w;
    }
    void setEuler ( const double& yaw, const double& pitch, const double& roll ) {
        double halfYaw = double ( yaw ) * double ( 0.5 );
        double halfPitch = double ( pitch ) * double ( 0.5 );
        double halfRoll = double ( roll ) * double ( 0.5 );
        double cosYaw = cos ( halfYaw );
        double sinYaw = sin ( halfYaw );
        double cosPitch = cos ( halfPitch );
        double sinPitch = sin ( halfPitch );
        double cosRoll = cos ( halfRoll );
        double sinRoll = sin ( halfRoll );
        setValue ( cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
                   cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
                   sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
                   cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw );
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Quaternion& copyFrom ( const T2 &src ) {
        x = src.x, y = src.y, z = src.z, w = src.w;
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo ( T2 &des ) const {
        des.x = x, des.y = y, des.z = z, des.w = w;
    }
    /** casts to vector4
     * @return vector4<double> cast
     **/
    ShmFw::Vector4<double> &asVector () {
        return (ShmFw::Vector4<double>&) *this;;
    }   
    /** casts to vector4
     * @return vector4<double> cast
     **/
    const ShmFw::Vector4<double> &asVector () const {
        return (ShmFw::Vector4<double>&) *this;;
    }  
    /** compares with within a expsilon range
     * @param o 
     * @param tolerance 
     * @see http://math.stackexchange.com/questions/90081/quaternion-distance
     **/
    bool equal( const Quaternion& q, double tolerance = 0.0001 ) const {
	 double dot = asVector().dot(q.asVector());
         double d = 1 - dot*dot;
	 return d < tolerance;
    }
    
    double getYaw() const {
        double opposite = 2 * z * w;
        double adjacent = 1 - 2 * z * z;
        if ( opposite >= 0 && adjacent >= 0 ) {
            return ( asin ( opposite ) );
        } else if ( opposite >= 0 && adjacent <= 0 ) {
            return ( M_PI - asin ( opposite ) );
        } else if ( opposite <= 0 && adjacent <= 0 ) {
            return ( M_PI - asin ( opposite ) );
        } else {
            return ( 2 * M_PI + asin ( opposite ) );
        }
    }
    
    /** computes the vector dot products
     * @param b 
     * @return dot product
     **/
    double dot(const Quaternion &b) const {
      return x*b.x + y*b.y + z*b.z + w*b.w;
    }
    
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "x", x );
        ar & make_nvp ( "y", y );
        ar & make_nvp ( "z", z );
        ar & make_nvp ( "w", w );
    }
};
};
#endif //SHARED_MEM_OBJECTS_QUATERNION_H


