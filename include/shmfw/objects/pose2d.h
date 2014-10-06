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
#ifndef SHARED_MEM_OBJECTS_POSE2D_H
#define SHARED_MEM_OBJECTS_POSE2D_H


#include <shmfw/objects/point2d.h>
#include <math.h>

namespace ShmFw {

class Pose2D : public ShmFw::BaseObject {
public:
    Point2D position;
    double orientation;
    Pose2D() : position(), orientation ( 0 ) {};
    Pose2D ( const Point2D &p, double o ) : position ( p ), orientation ( o ) {};
    Pose2D ( const Pose2D &p ) : position ( p.position ), orientation ( p.orientation ) {};
    Pose2D ( double _x, double _y, double _orientation ) : position ( _x,_y ), orientation ( _orientation ) {};
    Pose2D ( const ShmFw::Point2D &location, const ShmFw::Point2D &target ){
      setPose(location, target);
    };
    std::string getToStringFormat ( const std::string &format ) const {
        char buf[0xFF];
        sprintf ( buf, format.c_str(), position.x, position.y, orientation );
        return std::string ( buf );
    }
    std::string getToString() const {
        return getToStringFormat ( "[ [%lf, %lf], [%lf] ]" );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "[%lf,%lf],[%lf]", &position.x, &position.y, &orientation ) == EOF ) return false;
        return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const Pose2D &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Pose2D &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const Pose2D& o ) const {
        return position == o.position && orientation == o.orientation;
    }
    /** compares with within a tolerance
     * @param o 
     * @param tolerance 
     **/
    bool equal( const Pose2D& o, double tolerance = 0.0001 ) const {
         return position.equal(o.position, tolerance) && ((orientation - o.orientation) < tolerance); 
    }
    void setPose ( const ShmFw::Point2D &location, const ShmFw::Point2D &target ) {
        double dx = target.x - location.x;
        double dy = target.y - location.y;
        position = location;
        orientation = atan2 ( dy, dx );
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Pose2D& copyFrom ( const T2 &src) {
        position.copyFrom(src.position);
        orientation = src.orientation;
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        position.copyTo(des.position);
        des.orientation = orientation;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "position", position );
        ar & make_nvp ( "orientation", orientation );
    }
};


};
#endif //SHARED_MEM_OBJECTS_POSE2D_H

