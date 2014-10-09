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

#ifndef SHARED_MEM_VECTOR3
#define SHARED_MEM_VECTOR3

#include <shmfw/objects/vector2.h>

namespace ShmFw {


/** 3D Vector with x, y and z**/
template<typename T = float>
class Vector3 {
public:
    T x, y, z;
public:
    /** Constructor **/
    Vector3 () {}
    /** Constructor **/
    Vector3 ( const T& _x, const T& _y, const T& _z ) : x ( _x ), y ( _y ), z ( _z ) {}
    /** Constructor **/
    Vector3 ( const Vector3<T> &r ) : x ( r.x ), y ( r.y ), z ( r.z ) {}

    /** @return reference the first two elements **/
    const Vector2<T>& v3() const {
        return * ( ( Vector2<T>* ) this );
    }
    /** @return reference the first two elements **/
    Vector3<T>& v2() {
        return * ( ( Vector2<T>* ) this );
    }
    /** @return length of the vector **/
    T norm() const {
        return sqrt ( norm2() );
    }
    /** @return squared length of the vector **/
    T norm2() const {
        return x*x+y*y+z*z;
    }
    /** @return sum of x and y **/
    T sum() const {
        return x+y+z;
    }
    /** sets x and y to zero **/
    void clear() const {
        x = 0, y = 0, z = 0;
    }
     /**
      * sets 0 0 0
     **/
    void zero(){
      x = 0, y = 0, z = 0;
    }
    /** generates a random x and y between min and max
     * @ToDo the rand number genrator should be replaced by a better one
     * @param min
     * @param max
    **/
    void rand ( const T& min, const T& max ) {
        T d = max - min;
        x = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        y = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
        z = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
    }
    /** @return retuns vector with x*x and y*y **/
    Vector3<T> square() const {
        return Vector3<T> ( x*x, y*y, z*z );
    }
    /** normalizes the vecotr length to one **/
    Vector3<T>& normalize() {
        return ( *this ) /= norm();
    }
    /** Array operation
     * @param i entry to return
     * @return x as the first and y as the second entry
     **/
    T& operator[] ( int i ) {
        return ( &x ) [i];
    }
    /** Array operation const version
     * @param i entry to return
     * @return x as the first and y as the second entry
     **/
    const T& operator[] ( int i ) const {
        return ( &x ) [i];
    }
    /** Assignment opartion **/
    Vector3<T> operator = ( const Vector3<T>& v ) {
        x = v.x, y = v.y, z = v.z;
        return *this;
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Vector3<T>& copyFrom ( const T2 &src) {
        x = src.x, y = src.y, z = src.z;
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        des.x = x, des.y = y, des.z = z;
    }
    /** compares the vector with v and returns true if the endpoint difference
     * is smaller as the tollerance
     * @param v 
     * @param tolerance
     **/
    bool equal ( const Vector3<T>& v, T tolerance ) const {
        return (*this - v).norm() < (tolerance);
    }
    /** equal operator **/
    bool operator == ( const Vector3<T>& v ) const {
        return x == v.x && y == v.y && z == v.z;
    }
    /** smaller operator and equal **/
    bool operator <= ( const Vector3<T>& v ) const {
        return norm() <= v.norm();
    }
    /** bigger operator **/
    bool operator > ( const Vector3<T>& v ) const {
        return norm() > v.norm();
    }
    /** bigger operator and equal**/
    bool operator >= ( const Vector3<T>& v ) const {
        return norm() >= v.norm();
    }
    /** Not equal operator **/
    bool operator != ( const Vector3<T>& v ) const {
        return x != v.x || y != v.y || z != v.z;
    }
    /** Dot product or inner product **/
    T dot ( const Vector3<T>& v ) const {
        return x * v.x + y * v.y + z * v.z;
    }
    /** Dot product or inner product **/
    T operator * ( const Vector3<T>& v ) const {
        return dot ( v );
    }
    /** Crossproduct **/
    Vector3<T> operator^ ( const Vector3<T>& v ) const {
        return Vector3<T> ( y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x );
    }
    /** Crossproduct **/
    Vector3<T>& operator^= ( const Vector3<T>& v ) {
        return *this = ( *this ) ^ v;
    }
    /** Crossproduct **/
    Vector3<T> cross ( const Vector3<T>& v ) {
        return ( *this ) ^ v;
    }
    /** Substraction **/
    Vector3<T> operator -= ( const Vector3<T>& v ) {
        x -= v.x, y -= v.y, z -= v.z;
        return *this;
    }
    /** Substraction **/
    Vector3<T> operator - ( const Vector3<T>& v ) const {
        return Vector3<T> ( *this ) -= v;
    }
    /** Negation **/
    Vector3<T> operator-() const {
        return Vector3<T> ( -x, -y, -z );
    }
    /** Addition **/
    Vector3<T> operator += ( const Vector3<T>& v ) {
        x += v.x, y += v.y, z += v.z;
        return *this;
    }
    /** Addition **/
    Vector3<T> operator + ( const Vector3<T>& v ) const {
        return Vector3<T> ( *this ) += v;
    }

    /** Scalar addition */
    Vector3<T> add ( const T& scalar ) {
        x += scalar, y += scalar, z += scalar;
        return *this;
    }
    /** Scalar addition **/
    Vector3<T> operator += ( const int& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector3<T> operator += ( const float& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector3<T> operator += ( const double& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector3<T> operator + ( const int& scalar ) const {
        return Vector3<T> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vector3<T> operator + ( const float& scalar ) const {
        return Vector3<T> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vector3<T> operator + ( const double& scalar ) const {
        return Vector3<T> ( *this ).add ( scalar );
    }
    /** Scalar subtract */
    Vector3<T> subtract ( const T& scalar ) {
        x -= scalar, y -= scalar, z -= scalar;
        return *this;
    }
    /** Scalar substration */
    Vector3<T> operator -= ( const int& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector3<T> operator -= ( const float& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector3<T> operator -= ( const double& scalar ) const {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector3<T> operator - ( const int& scalar ) const {
        return Vector3<T> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vector3<T> operator - ( const float& scalar ) const {
        return Vector3<T> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vector3<T> operator - ( const double& scalar ) const {
        return Vector3<T> ( *this ).subtract ( scalar );
    }
    /** Scalar Muliplication */
    Vector3<T> multiplicate ( const T& scalar ) {
        x *= scalar, y *= scalar, z *= scalar;
        return *this;
    }
    /** Scalar Muliplication */
    Vector3<T> operator *= ( const int& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector3<T> operator *= ( const float& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector3<T> operator *= ( const double& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector3<T> operator * ( const int& scalar ) const {
        return Vector3<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector3<T> operator * ( const float& scalar ) const {
        return Vector3<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector3<T> operator * ( const double& scalar ) const {
        return Vector3<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Division */
    Vector3<T> divide ( const T& scalar ) {
        x /= scalar, y /= scalar, z /= scalar;
        return *this;
    }
    /** Scalar Division */
    Vector3<T> operator /= ( const int& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector3<T> operator /= ( const float& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector3<T> operator /= ( const double& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector3<T> operator / ( const int& scalar ) const {
        return Vector3<T> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vector3<T> operator / ( const float& scalar ) const {
        return Vector3<T> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vector3<T> operator / ( const double& scalar ) const {
        return Vector3<T> ( *this ).divide ( scalar );
    }
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << std::setw ( 12 ) << x <<  ", " << std::setw ( 12 ) << y <<  ", " << std::setw ( 12 ) << z << "]";
        return ss.str();
    }
    /** reads a human readable string [ x, y, z]
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
        if ( values.size() != 3 ) return 1;
        std::vector<std::string>::const_iterator it = values.begin();
        x = boost::lexical_cast<T> ( *it++ );
        y = boost::lexical_cast<T> ( *it++ );
        z = boost::lexical_cast<T> ( *it++ );
        return true;
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::Vector3<T> &o ) {
        os << "[" << o.x <<  ", " << o.y << ", " << o.z << "]";
        return os;
    }
    friend std::istream& operator >> (std::istream &is, const ShmFw::Vector3<T> &o)
    {
        return is;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "x", x );
        ar & make_nvp ( "y", y );
        ar & make_nvp ( "z", z );
    }
};
};


#endif //SHARED_MEM_VECTOR3


