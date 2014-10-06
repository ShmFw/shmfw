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
#ifndef SHARED_MEM_OBJECTS_POSE_H
#define SHARED_MEM_OBJECTS_POSE_H


#include <shmfw/objects/point.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/quaternion.h>

namespace ShmFw {

class Pose : public ShmFw::BaseObject {
public:
    Point position;
    Quaternion orientation;
    Pose() : position(), orientation() {};
    Pose ( const Point &p, const Quaternion &o ) : position ( p ), orientation ( o ) {};
    Pose ( const Pose &p ) : position ( p.position ), orientation ( p.orientation ) {};
    Pose ( const Pose2D &p, double z = 0 ) {
      setPose(p, z);
    };
    std::string getToStringFormat ( const std::string &format ) const {
        char buf[0xFF];
        sprintf ( buf, format.c_str(), position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w );
        return std::string ( buf );
    }
    std::string getToString() const {
        return getToStringFormat ( "[ [%lf, %lf, %lf], [%lf, %lf, %lf, %lf] ]" );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        start = data.find ( "[" );
        end = data.find ( "]", start );
        std::string pos = str.substr ( start+1, end-1 );
        start = data.find ( "[", end );
        end = data.find ( "]", start );
        std::string ori = str.substr ( start+1, end-1 );
        return position.setFromString ( pos ) && orientation.setFromString ( ori );
    }
    friend std::ostream& operator<< ( std::ostream &output, const Pose &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Pose &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const Pose& o ) const {
        return position == o.position && orientation == o.orientation;
    }
    /** compares with within a tolerance
     * @param o 
     * @param tolerance 
     **/
    bool equal( const Pose& o, double tolerance = 0.0001 ) const {
         return position.equal(o.position, tolerance) && orientation.equal(o.orientation, tolerance); 
    }
    /** Sets the pose values based on a 2D pose on the xy plane
     * @param pose data source
     * @param z data source
     **/
    void setPose ( const ShmFw::Pose2D &pose, double z = 0.) {
        position.x = pose.position.x;
        position.y = pose.position.y;
        position.z = z;
        orientation.setRotation ( ShmFw::Vector3<double> ( 0.,0.,1. ), pose.orientation );
    }
    /** Sets the pose values
     * @param p translation
     * @param o rotation
     **/
    void setPose( const Point &p, const Quaternion &o) {
        position = p, orientation = o;
    }
    /** Projects the 3D pose on the xy plane
     * @param des 2D pose
     **/
    ShmFw::Pose2D &getPose2D (ShmFw::Pose2D &des) {
        des.position.copyFrom(position);
	des.orientation = orientation.getYaw();
	return des;
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Pose& copyFrom ( const T2 &src) {
        position.copyFrom(src.position);
        orientation.copyFrom(src.orientation);
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        position.copyTo(des.position);
        orientation.copyTo(des.orientation);
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
#endif //SHARED_MEM_OBJECTS_POSE_H

