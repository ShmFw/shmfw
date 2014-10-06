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

#ifndef SHARED_MEM_PLANE
#define SHARED_MEM_PLANE

#include <iostream>
#include <shmfw/objects/vector3.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

namespace ShmFw {

template<typename> class Plane;

/** 3D PlaneEquation **/
template <class T = float>
class PlaneEquation  {
public:
    T v[4];
    /** Constructor **/
    PlaneEquation() {};
    /** Constructor **/
    PlaneEquation ( const PlaneEquation& r ) : v ( r.v ) {};

    void rand ( const T& min, const T&max ) {
        Vector3<T> p, n;
        p.rand ( min, max );
        n.rand ( min, max );
        create ( p, n );
    }
    /** Returns the normal vector to the plane.
     * The normal vector is reprecented by the first three elements of the plane equation
     * @return normal vector
     **/
    const Vector3<T>& normal() const {
        return * ( ( Vector3<T>* ) v );
    };
    /** Returns distance of a point to the plane
     * @return distance value
     **/
    T distanceToPlane ( const Vector3<T> &p ) const {
        return v[0]*p.x + v[1]*p.y + v[2]*p.z + v[3];
    }
    /** Assignment operator
     * @return this
     **/
    PlaneEquation operator = ( const PlaneEquation& e ) const {
        v[0] = e.v[0], v[1] = e.v[1], v[2] = e.v[2], v[3] = e.v[3];
        return *this;
    }
    /** comparison operator
     * @return this
     **/
    bool equal ( const PlaneEquation<T>& v, T tolerance = 0. ) const {
        return ( *this - v ).norm2() < ( tolerance*tolerance );
    }
    /** comparison operator
     * @return this
     **/
    bool operator == ( const PlaneEquation& e ) const {
        return v[0] == e.v[0] && v[1] == e.v[1] && v[2] == e.v[2] && v[3] == e.v[3];
    }
    /** sets all entries of the equation to zero  **/
    void clear() {
        v[0] = 1, v[1] = 0, v[2] = 0, v[3] = 0;
    }
    /** @return length of the vector **/
    T norm() const {
        return sqrt ( norm2 );
    }
    /** @return squared length of the vector **/
    T norm2() const {
        return v[0]*v[0]+v[1]*v[1]+v[2]*v[2]+v[3]*v[3];
    }
    /** computes the plane equation, based on three points
     * @param p1
     * @param p2
     * @param p3
     **/
    void create ( const Vector3<T> &p1, const Vector3<T> &p2, const Vector3<T> &p3 ) {
        Vector3<T> d2 = p2 - p1;
        Vector3<T> d3 = p3 - p1;
        normal() = d2^d3; // corss product
        normal() /= normal().norm();
        v[3] = - ( v[0]*p1.x + v[1]*p1.y + v[2]*p1.z );
    };
    /** computes the plane equation, based a point on the plane and the plane normal
     * @param p point on the plane
     * @param n plane normal
     **/
    void create ( const Vector3<T> &p, const Vector3<T> &n ) {
        normal() = n;
        normal() /= normal().norm();
        v[3] = - ( normal().dot ( p ) );
    };
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << std::setw ( 12 ) << v[0] <<  ", " << std::setw ( 12 ) << v[1] <<  ", " << std::setw ( 12 ) << v[2] <<  ", " << std::setw ( 12 ) << v[3] << "]";
        return ss.str();
    }
    /** reads a human readable string [ A, B, C, D]
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
        v[0] = boost::lexical_cast<T> ( *it++ );
        v[1] = boost::lexical_cast<T> ( *it++ );
        v[2] = boost::lexical_cast<T> ( *it++ );
        v[3] = boost::lexical_cast<T> ( *it++ );
        return true;
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::PlaneEquation<T> &o ) {
        os << "[" << o.v[0] <<  ", " << o.v[1] << ", " << o.v[2] << ", " << o.v[3] << "]";
        return os;
    };

protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "v", v );
    }
    /** Returns the normal vector to the plane.
     * The normal vector is reprecented by the first three elements of the plane equation
     * @return normal vector
     **/
    //friend class Plane<T>;
    Vector3<T>& normal() {
        return * ( ( Vector3<T>* ) v );
    };
};

/** 3D Plane (point and plane equation) **/
template <class T = float>
class Plane {
public:
    Vector3<T> p;
    PlaneEquation<T> eq;
    /** Constructor **/
    Plane() : p ( 0,0,0 ), eq () {}
    /** Constructor **/
    Plane ( const Plane<T> &r ) : p ( r.p ), eq ( r.eq ) {}
    /** creates a random plane **/
    void rand ( const T& min, const T&max ) {
        Vector3<T> n;
        p.rand ( min, max );
        n.rand ( min, max );
        eq.create ( p, n );
    }
    /** comparison operator
     * @return this
     **/
    bool operator == ( const Plane<T>& v ) const {
        return eq == v.eq && p == v.p;
    }
    /** Assignment operator
     * @return this
     **/
    Plane<T> operator = ( const Plane<T>& v ) const {
        p = v.p, eq = v.eq;
        return *this;
    }
    /** Finds a line plane intersection
     * @param p1 line start
     * @param p2 line end
     * @param intersection intersection point with plane
     * @param epsilon computation tolerance
     * @return true if there is an intersection
     **/
    bool intersectionLine ( const Vector3<T> &p1, const Vector3<T> &p2, Vector3<T> &intersection, float epsilon = 0.00001 ) const {
        Vector3<T> v = p2 - p1;
        T denominator = v*eq.normal();
        if ( fabs ( denominator ) < epsilon ) {
            return false;
        }
        T d = eq.distanceToPlane ( p1 );
        T u = d / ( eq.normal() * ( -v ) );
        intersection = p1 + v*u;
        return true;
    };
    /** computes the plane equation, based on three points
     * and remembers the point p1 as point on the plane
     * @param p1
     * @param p2
     * @param p3
     **/
    void create ( const Vector3<T> &p1, const Vector3<T> &p2, const Vector3<T> &p3 ) {
        eq.create ( p1, p2, p3 );
        p = p1;
    };
    /** computes the plane equation, based a point on the plane and the plane normal
     * and remembers the point p as point on the plane
     * @param p point on the plane
     * @param n plane normal
     **/
    void create ( const Vector3<T> &p0, const Vector3<T> &n ) {
        eq.create ( p, n );
        p = p0;
    };
    /** Write into a file **/
    void write ( const std::string &filename ) {
        std::ofstream ofs ( filename.c_str() );
        assert ( ofs.good() );
        boost::archive::xml_oarchive xml ( ofs );
        xml << boost::serialization::make_nvp ( "Plane", *this );
    }
    /** Read from a file **/
    void read ( const std::string &filename ) {
        std::ifstream ifs ( filename.c_str() );
        assert ( ifs.good() );
        boost::archive::xml_iarchive xml ( ifs );
        xml >> boost::serialization::make_nvp ( "Plane", *this );
    }
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "[ " << p.human_readable() <<  ", " << eq.human_readable() << "]";
        return ss.str();
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::Plane<T> &o ) {
        os << "[ " << o.p << "; " << o.eq << "]";
        return os;
    };
protected:
    friend class boost::serialization::access;
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "eq", eq );
        ar & make_nvp ( "p", p );
    }
};
};



#endif //SHARED_MEM_PLANE

