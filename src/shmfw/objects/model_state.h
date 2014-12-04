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
#ifndef SHARED_MEM_OBJECTS_MODEL_STATE_H
#define SHARED_MEM_OBJECTS_MODEL_STATE_H


#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/twist.h>
#include <boost/interprocess/containers/vector.hpp>

namespace ShmFw {

class ModelState : public ShmFw::BaseObject {
public:
    Pose pose;
    Twist twist;
    ModelState() : pose(), twist() {};
    ModelState ( const ModelState &w ) : pose ( w.pose ), twist ( w.twist ) {};
    ModelState ( const Pose &p, const Twist &v ) : pose ( p ), twist ( v ) {};
    ModelState ( const Pose &p ) : pose ( p ), twist () {};
    ModelState ( const Pose2D &p, double z = 0) : pose ( p, z ), twist () {};
    std::string getToString ( ) const {
        char buf[0x1FF];
        sprintf ( buf, "[[[%4.2lf, %4.2lf, %4.2lf], [%4.3lf, %4.3lf, %4.3lf, %4.3lf]], [[%4.2lf, %4.2lf, %4.2lf], [%4.2lf, %4.2lf, %4.2lf]]]",
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
	boost::erase_all(data, " ");
	if(sscanf(data.c_str(), "[[%lf,%lf,%lf],[%lf,%lf,%lf,%lf]],[[%lf,%lf,%lf],[%lf,%lf,%lf]]",
                  &pose.position.x,    &pose.position.y,    &pose.position.z,
                  &pose.orientation.x, &pose.orientation.y, &pose.orientation.z, &pose.orientation.w,
                  &twist.linear.x,     &twist.linear.y,     &twist.linear.z, 
	          &twist.angular.x,    &twist.angular.y,    &twist.angular.z ) == EOF) return false; 
	return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const ModelState &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, ModelState &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const ModelState& o ) const {
        return pose == o.pose && twist == o.twist;
    }
    void setValues ( const Pose &p, const Twist &v ) {
        pose = p, twist = v;
    }
    void setPose ( const Pose &p ) {
        pose = p;
    }
    void setPose ( const Pose2D &p, double z = 0) {
        pose.setPose(p, z);
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    ModelState& copyFrom ( const T2 &src) {
        pose.copyFrom(src.pose);
        twist.copyFrom(src.twist);
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        pose.copyTo(des.pose);
        twist.copyTo(des.twist);
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "pose", pose );
        ar & make_nvp ( "twist", twist );
    }
};

template <template<typename...> class Allocator>
using ModelStates = boost::interprocess::vector<ModelState, Allocator<ModelState> >;

};
#endif //SHARED_MEM_OBJECTS_MODEL_STATE_H

