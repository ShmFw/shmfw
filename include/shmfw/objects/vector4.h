/***************************************************************************
 *   Software License Agreement (BSD License)                              *  
 *   Copyright (C) 2012 by Markus Bader <markus.yader@tuwien.ac.at>        *
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

#ifndef SHARED_MEM_VECTOR4
#define SHARED_MEM_VECTOR4

#include <iostream>
#include <shmfw/objects/vector3.h>

namespace ShmFw {


/** 4D Vector **/
template<typename T = float>
class Vector4 {
public:
    T x, y, z, w;
public:
    /** Constructor **/
    Vector4 () {}
    /** Constructor **/
    Vector4 ( const T& _x, const T& _y, const T& _z, const T& _w ) : x ( _x ), y ( _y ), z ( _z ), w ( _w ) {}
    /** Constructor **/
    Vector4 ( const Vector4<T> &r ) : x ( r.x ), y ( r.y ), z ( r.z ), w ( r.w ) {}
 
    /** @return reference the first two elements **/
    const Vector3<T>& v3() const {
        return * ( ( Vector3<T>* ) this );
    }
    /** @return reference the first two elements **/
    Vector3<T>& v3() {
        return * ( ( Vector3<T>* ) this );
    }
    /** @return reference the first two elements **/
    const Vector2<T>& v2() const {
        return * ( ( Vector2<T>* ) this );
    }
    /** @return reference the first two elements **/
    Vector2<T>& v2() {
        return * ( ( Vector2<T>* ) this );
    }
    /** @return length of the vector **/
    T norm() const {
        return ::sqrt ( norm2() );
    }
    /** @return squared length of the vector **/
    T norm2() const {
        return x*x+y*y+z*z+w*w;
    }
    /** @return sum y **/
    T sum() const {
        return x+y+z+w;
    }
    /** sets zeros **/
    void clear() const {
        x = 0, y = 0, z = 0, w = 0;
    }
    /** generates random values between min and max
     * @ToDo the rand number genrator should be replaced by a better one
     * @param min
     * @param max
    **/
    void rand ( const T& min, const T& max ) {
        T d = max - min;
        x = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        y = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        z = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        w = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
    }
    /** @return retuns vector with x*x and y*y **/
    Vector4<T> square() const {
        return Vector4<T> ( x*x, y*y, z*z, w*w );
    }
    /** normalizes the vecotr length to one **/
    Vector4<T>& normalize() {
        return ( *this ) /= norm();
    }
    /** Array operation
     * @param i entry to return
     * @return reference to the element
     **/
    T& operator[] ( int i ) {
        return ( &x ) [i];
    }
    /** Array operation const version
     * @param i entry to return
     * @return reference to the element
     **/
    const T& operator[] ( int i ) const {
        return ( &x ) [i];
    }
    /** Assignment opartion **/
    Vector4<T> operator = ( const Vector4<T>& v ) {
        x = v.x, y = v.y, z = v.z, w = v.w;
        return *this;
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Vector4<T>& copyFrom ( const T2 &src) {
        x = src.x, y = src.y, z = src.z, w = src.w;
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        des.x = x, des.y = y, des.z = z, des.w = w;
    }
    /** compares the vector with v and returns true if the endpoint difference
     * is smaller as the tollerance
     * @param v 
     * @param tolerance
     **/
    bool equal ( const Vector4<T>& v, T tolerance ) const {
        return (*this - v).norm() < (tolerance);
    }
    /** equal operator **/
    bool operator == ( const Vector4<T>& v ) const {
        return x == v.x && y == v.y && z == v.z && w == v.w;
    }
    /** smaller operator and equal **/
    bool operator <= ( const Vector4<T>& v ) const {
        return norm() <= v.norm();
    }
    /** bigger operator **/
    bool operator > ( const Vector4<T>& v ) const {
        return norm() > v.norm();
    }
    /** bigger operator and equal**/
    bool operator >= ( const Vector4<T>& v ) const {
        return norm() >= v.norm();
    }
    /** Not equal operator **/
    bool operator != ( const Vector4<T>& v ) const {
        return x != v.x || y != v.y || z != v.z || w != v.w;
    }
    /** Dot product or inner product **/
    T dot ( const Vector4<T>& v ) const {
        return x * v.x + y * v.y + z * v.z + w * v.w;
    }
    /** Dot product or inner product **/
    T operator * ( const Vector4<T>& v ) const {
        return dot ( v );
    }
    /** Crossproduct **/
    Vector4<T> operator^ ( const Vector4<T>& v ) const {
	std::cerr << "Vector4<T> croos product not yet implemented";
        return Vector4<T> (  );
    }
    /** Crossproduct **/
    Vector4<T>& operator^= ( const Vector4<T>& v ) {
        return *this = ( *this ) ^ v;
    }
    /** Crossproduct **/
    Vector4<T> cross ( const Vector4<T>& v ) {
        return ( *this ) ^ v;
    }
    /** Substraction **/
    Vector4<T> operator -= ( const Vector4<T>& v ) {
        x -= v.x, y -= v.y, z -= v.z, w -= v.w;
        return *this;
    }
    /** Substraction **/
    Vector4<T> operator - ( const Vector4<T>& v ) const {
        return Vector4<T> ( *this ) -= v;
    }
    /** Negation **/
    Vector4<T> operator-() const {
        return Vector4<T> ( -x, -y, -z, -w );
    }
    /** Addition **/
    Vector4<T> operator += ( const Vector4<T>& v ) {
        x += v.x, y += v.y, z += v.z, w += v.w;
        return *this;
    }
    /** Addition **/
    Vector4<T> operator + ( const Vector4<T>& v ) const {
        return Vector4<T> ( *this ) += v;
    }

    /** Scalar addition */
    Vector4<T> add ( const T& scalar ) {
        x += scalar, y += scalar, z += scalar, w += scalar;
        return *this;
    }
    /** Scalar addition **/
    Vector4<T> operator += ( const int& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector4<T> operator += ( const float& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector4<T> operator += ( const double& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector4<T> operator + ( const int& scalar ) const {
        return Vector4<T> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vector4<T> operator + ( const float& scalar ) const {
        return Vector4<T> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vector4<T> operator + ( const double& scalar ) const {
        return Vector4<T> ( *this ).add ( scalar );
    }
    /** Scalar subtract */
    Vector4<T> subtract ( const T& scalar ) {
        x -= scalar, y -= scalar, z -= scalar, w -= scalar;
        return *this;
    }
    /** Scalar substration */
    Vector4<T> operator -= ( const int& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector4<T> operator -= ( const float& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector4<T> operator -= ( const double& scalar ) const {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector4<T> operator - ( const int& scalar ) const {
        return Vector4<T> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vector4<T> operator - ( const float& scalar ) const {
        return Vector4<T> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vector4<T> operator - ( const double& scalar ) const {
        return Vector4<T> ( *this ).subtract ( scalar );
    }
    /** Scalar Muliplication */
    Vector4<T> multiplicate ( const T& scalar ) {
        x *= scalar, y *= scalar, z *= scalar, w *= scalar;
        return *this;
    }
    /** Scalar Muliplication */
    Vector4<T> operator *= ( const int& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector4<T> operator *= ( const float& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector4<T> operator *= ( const double& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector4<T> operator * ( const int& scalar ) const {
        return Vector4<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector4<T> operator * ( const float& scalar ) const {
        return Vector4<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector4<T> operator * ( const double& scalar ) const {
        return Vector4<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Division */
    Vector4<T> divide ( const T& scalar ) {
        x /= scalar, y /= scalar, z /= scalar, w /= scalar;
        return *this;
    }
    /** Scalar Division */
    Vector4<T> operator /= ( const int& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector4<T> operator /= ( const float& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector4<T> operator /= ( const double& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector4<T> operator / ( const int& scalar ) const {
        return Vector4<T> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vector4<T> operator / ( const float& scalar ) const {
        return Vector4<T> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vector4<T> operator / ( const double& scalar ) const {
        return Vector4<T> ( *this ).divide ( scalar );
    }
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << std::setw ( 12 ) << x <<  ", " << std::setw ( 12 ) << y <<  ", " << std::setw ( 12 ) << z <<  ", " << std::setw ( 12 ) << w << "]";
        return ss.str();
    }
    /** reads a human readable string [ x, y, z, w]
     * @param str string to decipher
     * @param comma string ","
     * @param bracketOpen string "["
     * @param bracketClose string "]"
     * @return true on suggess
     **/
    bool human_readable ( const std::string &str, std::string comma = ",", std::string bracketOpen = "[", std::string bracketClose = "]" ) {
        if ( str.empty() ) return 1;
        std::string valueString;
        size_t  start = str.find_first_of ( bracketOpen );
        if ( start == std::string::npos ) return false;
        size_t end = str.find_first_of ( bracketClose, start );
        if ( end == std::string::npos ) return false;
	
        std::string tmp = str.substr ( start+1, end-start-1 );
        boost::erase_all ( tmp, " " );
        std::vector<std::string> values;
        boost::split ( values, tmp, boost::is_any_of ( comma ) );
        if ( values.size() != 4 ) return 1;
        std::vector<std::string>::const_iterator it = values.begin();	
        x = boost::lexical_cast<T> ( *it++ );
        y = boost::lexical_cast<T> ( *it++ );
        z = boost::lexical_cast<T> ( *it++ );
        w = boost::lexical_cast<T> ( *it++ );
        return true;
    }
friend std::ostream &operator << ( std::ostream &os, const ShmFw::Vector4<T> &o ) {
    os << "[" << o.x <<  ", " << o.y << ", " << o.z  << ", " << o.w <<"]";
    return os;
};
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


#endif //SHARED_MEM_VECTOR3


