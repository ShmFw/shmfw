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

#ifndef SHARED_MEM_VEC
#define SHARED_MEM_VEC

#include <iostream>
#include <shmfw/objects/vector3.h>

namespace ShmFw {


/** Vector **/
template<typename T, unsigned int N>
class Vec {
public:
    T v[N];
public:
    /** Constructor **/
    Vec () {}
    /** Constructor **/
    Vec ( const T& v0, const T& v1 ) {
        copyFrom ( v0,v1 );
    }
    Vec ( const T& v0, const T& v1, const T& v2 ) {
        copyFrom ( v0,v1,v2 );
    }
    Vec ( const T& v0, const T& v1, const T& v2, const T& v3 ) {
        copyFrom ( v0,v1,v2,v3 );
    }
    Vec ( const T& v0, const T& v1, const T& v2, const T& v3, const T& v4 ) {
        copyFrom ( v0,v1,v2,v3,v4 );
    }
    Vec ( const T& v0, const T& v1, const T& v2, const T& v3, const T& v4, const T& v5 ) {
        copyFrom ( v0,v1,v2,v3,v4,v5 );
    }
    Vec ( const T& v0, const T& v1, const T& v2, const T& v3, const T& v4, const T& v5, const T& v6 ) {
        copyFrom ( v0,v1,v2,v3,v4,v5,v6 );
    }
    Vec ( const T& v0, const T& v1, const T& v2, const T& v3, const T& v4, const T& v5, const T& v6, const T& v7 ) {
        copyFrom ( v0,v1,v2,v3,v4,v5,v6,v7 );
    }
    /** Constructor **/
    Vec ( const Vec<T, N> &_v ) {
        for ( unsigned int i = 0; i < N; i++ ) v[i] = _v.v[i];
    }

    /** @return reference the first three elements **/
    const Vector3<T>& v3() const {
        return * ( ( Vector3<T>* ) this );
    }
    /** @return reference the first three elements **/
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
        T n2 = 0;
        for ( unsigned int i = 0; i < N; i++ ) n2 += v[i]*v[i];
        return n2;
    }
    /** @return sum all elements up **/
    T sum() const {
        T s = 0;
        for ( unsigned int i = 0; i < N; i++ ) s += v[i];
        return s;
    }
    /** sets all to 0 **/
    void clear ( bool homogeneous = true ) const {
        for ( unsigned int i = 0; i < N; i++ ) v[i] = 0;
    }
    /** generates a random x and y between min and max
     * @ToDo the rand number genrator should be replaced by a better one
     * @param min
     * @param max
    **/
    void rand ( const T& min, const T& max, bool homogen = true ) {
        T d = max - min;
        for ( unsigned int i = 0; i < N; i++ ) v[i] = ( ( T ) ::rand() ) * d / ( T ) RAND_MAX + min;
    }
    /** @return retuns vector with x*x and y*y **/
    Vec<T, N> sqrt() const {
        Vec<T, N> vec;
        for ( unsigned int i = 0; i < N; i++ ) vec[i] = v[i]*v[i];
        return vec;
    }
    /** normalizes the vecotr length to one **/
    Vec<T, N>& normalize() {
        return ( *this ) /= norm();
    }
    /** pointer to the first ellement
     * @param i entry to return
     * @return pointer
     **/
    T* ptr() {
        return &v;
    }
    /** pointer to the first ellement
     * @param i entry to return
     * @return pointer
     **/
    const T* ptr() const {
        return &v;
    }
    /** pointer to an ellement
     * @param i entry to return
     * @return pointer
     **/
    T* ptr ( int i ) {
        return &v[i];
    }
    /** pointer to an ellement
     * @param i entry to return
     * @return pointer
     **/
    const T* ptr ( int i ) const {
        return &v[i];
    }
    /** Array operation
     * @param i entry to return
     * @return x as the first and y as the second entry
     **/
    T& operator[] ( int i ) {
        return v[i];
    }
    /** Array operation const version
     * @param i entry to return
     * @return x as the first and y as the second entry
     **/
    const T& operator[] ( int i ) const {
        return v[i];
    }
    /** Assignment opartion **/
    Vec<T,N> operator = ( const Vec<T,N>& _v ) {
        for ( unsigned int i = 0; i < N; i++ ) v[i] = _v.v[i];
        return *this;
    }
    /** Copies data from an array
     * @param v data source
     **/
    Vec<T,N> copyFrom ( const T *v ) {
        for ( T *p=&v[0]; p!=&v[N]; p++ ) *p = *v++;
        return *this;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0 ) {
        v[0] = v0;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1 ) {
        v[0] = v0, v[1] = v1;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1, const T &v2 ) {
        v[0] = v0, v[1] = v1, v[2] = v2;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1, const T &v2, const T &v3 ) {
        v[0] = v0, v[1] = v1, v[2] = v2, v[3] = v3;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1, const T &v2, const T &v3, const T &v4 ) {
        v[0] = v0, v[1] = v1, v[2] = v2, v[3] = v3, v[4] = v4;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1, const T &v2, const T &v3, const T &v4, const T &v5 ) {
        v[0] = v0, v[1] = v1, v[2] = v2, v[3] = v3, v[4] = v4, v[5] = v5;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1, const T &v2, const T &v3, const T &v4, const T &v5, const T &v6 ) {
        v[0] = v0, v[1] = v1, v[2] = v2, v[3] = v3, v[4] = v4, v[5] = v5, v[6] = v6;
    }
    /** Copies data
     * @param v data source
     **/
    void copyFrom ( const T &v0, const T &v1, const T &v2, const T &v3, const T &v4, const T &v5, const T &v6, const T &v7 ) {
        v[0] = v0, v[1] = v1, v[2] = v2, v[3] = v3, v[4] = v4, v[5] = v5, v[6] = v6, v[7] = v7;
    }
    /** Copies data to an array
     * @param v data target
     **/
    void copyTo ( T *v ) const {
        for ( const T *p=&v[0]; p!=&v[N]; p++ ) *v++ = *p;
    }
    /** compares the vector with v and returns true if the endpoint difference
     * is smaller as the tollerance
     * @param v 
     * @param tolerance
     **/
    bool equal ( const Vec<T,N>& _v, T tolerance) const {
        return ( *this - _v ).norm() < ( tolerance);
    }
    /** equal operator **/
    bool operator == ( const Vec<T,N>& _v ) const {
        for ( unsigned int i = 0; i < N; i++ ) {
            if ( v[i] != _v[i] ) return false;
        }
        return true;
    }
    /** smaller operator and equal **/
    bool operator <= ( const Vec<T,N>& _v ) const {
        return norm() <= _v.norm();
    }
    /** bigger operator **/
    bool operator > ( const Vec<T,N>& _v ) const {
        return norm() > _v.norm();
    }
    /** bigger operator and equal**/
    bool operator >= ( const Vec<T,N>& _v ) const {
        return norm() >= _v.norm();
    }
    /** Not equal operator **/
    bool operator != ( const Vec<T,N>& _v ) const {
        for ( unsigned int i = 0; i < N; i++ ) {
            if ( v[i] != _v[i] ) return true;
        }
        return false;
    }
    /** Dot product or inner product **/
    T dot ( const Vec<T,N>& _v ) const {
        T d = 0;
        for ( unsigned int i = 0; i < N; i++ ) d += v[i]*_v[i];
        return d;
    }
    /** Dot product or inner product **/
    T operator * ( const Vec<T,N>& _v ) const {
        return dot ( _v );
    }
    /** Crossproduct
     * @todo
     **/
    Vec<T,N> operator^ ( const Vec<T,N>& _v ) const {
        std::cerr << "Vec<T,N> croos product not yet implemented";
        return Vec<T,N> ( );
    }
    /** Crossproduct **/
    Vec<T,N>& operator^= ( const Vec<T,N>& _v ) {
        return *this = ( *this ) ^ v;
    }
    /** Crossproduct **/
    Vec<T,N> cross ( const Vec<T,N>& _v ) {
        return ( *this ) ^ v;
    }
    /** Substraction **/
    Vec<T,N> operator -= ( const Vec<T,N>& _v ) {
        for ( unsigned int i = 0; i < N; i++ ) v[i] -= _v.v[i];
        return *this;
    }
    /** Substraction **/
    Vec<T,N> operator - ( const Vec<T,N>& _v ) const {
        return Vec<T,N> ( *this ) -= _v;
    }
    /** Negation **/
    Vec<T,N> operator-() const {
        Vec<T,N> vec;
        for ( unsigned int i = 0; i < N; i++ ) vec.v[i] = -v[i];
        return vec;
    }
    /** Addition **/
    Vec<T,N> operator += ( const Vec<T,N>& _v ) {
        for ( unsigned int i = 0; i < N; i++ ) v[i]  += _v[i];
        return *this;
    }
    /** Addition **/
    Vec<T,N> operator + ( const Vec<T,N>& _v ) const {
        return Vec<T,N> ( *this ) += _v;
    }

    /** Scalar addition */
    Vec<T,N> add ( const T& scalar ) {
        for ( T *p=&v[0]; p!=&v[N]; p++ ) *p += scalar;
        return *this;
    }
    /** Scalar addition **/
    Vec<T,N> operator += ( const int& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vec<T,N> operator += ( const float& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vec<T,N> operator += ( const double& scalar ) {
        return add ( scalar );
    }
    /** Scalar addition **/
    Vec<T,N> operator + ( const int& scalar ) const {
        return Vec<T,N> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vec<T,N> operator + ( const float& scalar ) const {
        return Vec<T,N> ( *this ).add ( scalar );
    }
    /** Scalar addition **/
    Vec<T,N> operator + ( const double& scalar ) const {
        return Vec<T,N> ( *this ).add ( scalar );
    }
    /** Scalar subtract */
    Vec<T,N> subtract ( const T& scalar ) {
        for ( T *p=&v[0]; p!=&v[N]; p++ ) *p -= scalar;
        return *this;
    }
    /** Scalar substration */
    Vec<T,N> operator -= ( const int& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vec<T,N> operator -= ( const float& scalar ) {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vec<T,N> operator -= ( const double& scalar ) const {
        return subtract ( scalar );
    }
    /** Scalar substration **/
    Vec<T,N> operator - ( const int& scalar ) const {
        return Vec<T,N> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vec<T,N> operator - ( const float& scalar ) const {
        return Vec<T,N> ( *this ).subtract ( scalar );
    }
    /** Scalar substration **/
    Vec<T,N> operator - ( const double& scalar ) const {
        return Vec<T,N> ( *this ).subtract ( scalar );
    }
    /** Scalar Muliplication */
    Vec<T,N> multiplicate ( const T& scalar ) {
        for ( T *p=&v[0]; p!=&v[N]; p++ ) *p *= scalar;
        return *this;
    }
    /** Scalar Muliplication */
    Vec<T,N> operator *= ( const int& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vec<T,N> operator *= ( const float& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vec<T,N> operator *= ( const double& scalar ) {
        return multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vec<T,N> operator * ( const int& scalar ) const {
        return Vec<T,N> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vec<T,N> operator * ( const float& scalar ) const {
        return Vec<T,N> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Muliplication */
    Vec<T,N> operator * ( const double& scalar ) const {
        return Vec<T,N> ( *this ).multiplicate ( scalar );
    }
    /** Scalar Division */
    Vec<T,N> divide ( const T& scalar ) {
        for ( unsigned int i = 0; i < N; i++ ) v[i] /= scalar;
        return *this;
    }
    /** Scalar Division */
    Vec<T,N> operator /= ( const int& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vec<T,N> operator /= ( const float& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vec<T,N> operator /= ( const double& scalar ) {
        return divide ( scalar );
    }
    /** Scalar Division */
    Vec<T,N> operator / ( const int& scalar ) const {
        return Vec<T,N> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vec<T,N> operator / ( const float& scalar ) const {
        return Vec<T,N> ( *this ).divide ( scalar );
    }
    /** Scalar Division */
    Vec<T,N> operator / ( const double& scalar ) const {
        return Vec<T,N> ( *this ).divide ( scalar );
    }
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << std::setw ( 12 ) << v[0];
        for ( const T *p=&v[1]; p!=&v[N]; p++ )  ss <<  ", " << std::setw ( 12 ) << *p;
        ss << "]";
        return ss.str();
    }
    /** reads a human readable string
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
        if ( values.size() != N ) return 1;
        std::vector<std::string>::const_iterator it = values.begin();
        for ( T *p=&v[0]; p!=&v[N]; p++ ) *p = boost::lexical_cast<T> ( *it++ );
        return true;
    }
friend  std::ostream &operator << ( std::ostream &os, const ShmFw::Vec<T,N> &o ) {
    os << "[" << o.v[0];
    for ( const T *p=&o.v[1]; p!=&o.v[N]; p++ ) os << ", " << *p;
    os  << "]";
    return os;
};
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "v", v );
    }
};
};


#endif //SHARED_MEM_VEC


