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

#ifndef SHARED_MEM_VECTOR2
#define SHARED_MEM_VECTOR2

#include <iostream>
#include <valarray>
#include <vector>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

namespace ShmFw {


/** 2D Vector with x and y **/
template<typename T = float>
class Vector2 {
public:
    T x, y;
public:
    /** Constructor **/
    Vector2 () {}
    /** Constructor **/
    Vector2 ( T _x, T _y ) : x ( _x ), y ( _y ) {}
    /** Constructor **/
    Vector2 ( const Vector2 &r ) : x ( r.x ), y ( r.y ) {}

    /** @return length of the vector **/
    T norm() const {
        return sqrt ( norm2() );
    }
    /** @return squared length of the vector **/
    T norm2() const {
        return x*x+y*y;
    }
    /** @return sum of x and y **/
    T sum() const {
        return x+y;
    }
    /** sets x and y to zero **/
    void clear() const {
        x = 0, y = 0;
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
    }
    /** @return retuns vector with x*x and y*y **/
    Vector2<T> square() const {
        return Vector2<T> ( x*x, y*y );
    }
    /** normalizes the vecotr length to one **/
    Vector2<T>& normalize() {
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
    Vector2<T> operator = ( const Vector2<T>& v ) {
        x = v.x, y = v.y;
        return *this;
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Vector2<T>& copyFrom ( const T2 &src) {
        x = src.x, y = src.y;
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        des.x = x, des.y = y;
    }
    /** compares the vector with v and returns true if the endpoint difference
     * is smaller as the tollerance
     * @param v 
     * @param tolerance
     **/
    bool equal ( const Vector2<T>& v, T tolerance) const {
        return ( *this - v ).norm() < ( tolerance );
    }
    /** equal operator **/
    bool operator == ( const Vector2<T>& v ) const {
        return x == v.x && y == v.y;
    }
    /** smaller operator **/
    bool operator < ( const Vector2<T>& v ) const {
        return norm() < v.norm();
    }
    /** smaller operator and equal **/
    bool operator <= ( const Vector2<T>& v ) const {
        return norm() <= v.norm();
    }
    /** bigger operator **/
    bool operator > ( const Vector2<T>& v ) const {
        return norm() > v.norm();
    }
    /** bigger operator and equal**/
    bool operator >= ( const Vector2<T>& v ) const {
        return norm() >= v.norm();
    }
    /** Not equal operator **/
    bool operator != ( const Vector2<T>& v ) const {
        return x != v.x || y != v.y;
    }
    /** Dot product or inner product **/
    T operator * ( const Vector2<T>& v ) const {
        return x * v.x + y * v.y;
    }
    /** Dot product or inner product **/
    T dot ( const Vector2<T>& v ) const {
        return x * v.x + y * v.y;
    }
    /** Substraction **/
    Vector2<T> operator -= ( const Vector2<T>& v ) {
        x -= v.x, y -= v.y;
        return *this;
    }
    /** Substraction **/
    Vector2<T> operator - ( const Vector2<T>& v ) const {
        return Vector2<T> ( *this ) -= v;
    }
    /** Negation **/
    Vector2<T> operator-() const {
        return Vector2<T> ( -x, -y );
    }
    /** Addition **/
    Vector2<T> operator += ( const Vector2<T>& v ) {
        x += v.x, y += v.y;
        return *this;
    }
    /** Addition **/
    Vector2<T> operator + ( const Vector2<T>& v ) const {
        return Vector2<T> ( *this ) += v;
    }
    /** Scalar addition */
    Vector2<T> add ( const T& scalar ) {
        x += scalar, y += scalar;
        return *this;
    }
    /** Scalar addition **/
    Vector2<T> operator += ( const int& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector2<T> operator += ( const float& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector2<T> operator += ( const double& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vector2<T> operator + ( const int& scalar ) const {
        return Vector2<T> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vector2<T> operator + ( const float& scalar ) const {
        return Vector2<T> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vector2<T> operator + ( const double& scalar ) const {
        return Vector2<T> ( *this ).add ( scalar );
    }
    /** Scalar subtract */
    Vector2<T> subtract ( const T& scalar ) {
        x -= scalar, y -= scalar;
        return *this;
    }
    /** Scalar substration */
    Vector2<T> operator -= ( const int& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector2<T> operator -= ( const float& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector2<T> operator -= ( const double& scalar ) const {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vector2<T> operator - ( const int& scalar ) const {
        return Vector2<T> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vector2<T> operator - ( const float& scalar ) const {
        return Vector2<T> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vector2<T> operator - ( const double& scalar ) const {
        return Vector2<T> ( *this ).subtract ( scalar );
    }
    /** Scalar Muliplication */
    Vector2<T> multiplicate ( const T& scalar ) {
        x *= scalar, y *= scalar;
        return *this;
    }
    /** Scalar Muliplication */
    Vector2<T> operator *= ( const int& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector2<T> operator *= ( const float& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector2<T> operator *= ( const double& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector2<T> operator * ( const int& scalar ) const {
        return Vector2<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector2<T> operator * ( const float& scalar ) const {
        return Vector2<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vector2<T> operator * ( const double& scalar ) const {
        return Vector2<T> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Division */
    Vector2<T> divide ( const T& scalar ) {
        x /= scalar, y /= scalar;
        return *this;
    }
    /** Scalar Division */
    Vector2<T> operator /= ( const int& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector2<T> operator /= ( const float& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector2<T> operator /= ( const double& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vector2<T> operator / ( const int& scalar ) const {
        return Vector2<T> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vector2<T> operator / ( const float& scalar ) const {
        return Vector2<T> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vector2<T> operator / ( const double& scalar ) const {
        return Vector2<T> ( *this ).divide ( scalar );
    }
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << std::setw ( 12 ) << x <<  ", " << std::setw ( 12 ) << y << "]";
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
        if ( values.size() != 2 ) return 1;
        std::vector<std::string>::const_iterator it = values.begin();
        x = boost::lexical_cast<T> ( *it++ );
        y = boost::lexical_cast<T> ( *it++ );
        return true;
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::Vector2<T> &o ) {
        os << "[" << o.x << ", " << o.y << "]";
        return os;
    };
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "x", x );
        ar & make_nvp ( "y", y );
    }
};
};



#endif //SHARED_MEM_VECTOR2

