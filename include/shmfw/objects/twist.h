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
#ifndef SHARED_MEM_OBJECTS_TWIST_H
#define SHARED_MEM_OBJECTS_TWIST_H

#include <shmfw/objects/vector3.h>
#include <boost/graph/graph_concepts.hpp>

namespace ShmFw {

class Twist {
public:
    Vector3<double> linear;
    Vector3<double> angular;
    Twist() : linear(), angular() {};
    Twist ( const Twist &p ) : linear ( p.linear ), angular ( p.angular ) {};
    Twist ( Vector3<double> linear, Vector3<double> angular ) : linear ( linear ), angular ( angular ) {};
    Twist ( double vx, double vy, double vz, double wx, double wy, double wz ) : linear ( vx, vy, vz ), angular ( wx, wy, wz ) {};
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ [%lf, %lf, %lf], [ %lf, %lf, %lf] ]", linear.x, linear.y, linear.z, angular.x, angular.y, angular.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "[%lf,%lf,%lf],[%lf,%lf,%lf]", &linear.x, &linear.y, &linear.z, &angular.x, &angular.y, &angular.z ) == EOF ) return false;
        return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const Twist &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Twist &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const Twist& v ) const {
        return linear == v.linear && angular == v.angular;
    }
    /** compares with within a expsilon range
     * @param o
     * @param tolerance
     **/
    bool equal ( const Twist& o, double tolerance = 0.0001 ) const {
        return linear.equal ( o.linear, tolerance ) && angular.equal ( o.angular, tolerance );
    }
    template<typename T>
    void copyTo ( T& des ) const {
        linear.copyTo ( des.linear );
        angular.copyTo ( des.angular );
    }
    template<typename T>
    Twist& copyFrom ( const T& src ) {
        linear.copyFrom ( src.linear );
        angular.copyFrom ( src.angular );
        return *this;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "linear", linear );
        ar & make_nvp ( "angular", angular );
    }
};

class Twist2D : public Twist {
public:
    Twist2D() : Twist() {};
    Twist2D ( const Twist2D &p ) : Twist ( p ) {};
    Twist2D ( double v, double w ) : Twist ( v, 0, 0, 0, 0, w ) {};
    const double &v() const{
      return linear.x;
    }
    double &v(){
      return linear.x;
    }    
    const double &w() const{
      return angular.z;
    }
    double &w(){
      return angular.z;
    }
};

};
#endif //SHARED_MEM_OBJECTS_TWIST_H

