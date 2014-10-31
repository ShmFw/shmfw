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
#ifndef SHARED_MEM_OBJECTS_AGENT_STATE_H
#define SHARED_MEM_OBJECTS_AGENT_STATE_H


#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/twist.h>

namespace ShmFw {

class AgentState : public ShmFw::BaseObject {
public:
    unsigned long trip_recorder;  /// distance moved
    double sub_meter_trip_recorder;  /// it should be allways <= 1
    Twist current; /// this are the current executed controls
    Twist target;  /// this are the desigred controls    
    std::string getToString ( ) const {
        char buf[0x1FF];
        sprintf ( buf, "[[%lu, %6.5lf], [[%4.2lf, %4.2lf, %4.2lf], [%4.2lf, %4.2lf, %4.2lf]], [[%4.2lf, %4.2lf, %4.2lf], [%4.2lf, %4.2lf, %4.2lf]]]",
                  trip_recorder, sub_meter_trip_recorder, 
		  current.linear.x, current.linear.y, current.linear.z, current.angular.x, current.angular.y, current.angular.z , 
		  target.linear.x, target.linear.y, target.linear.z, target.angular.x, target.angular.y, target.angular.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
	boost::erase_all(data, " ");
	if(sscanf(data.c_str(), "[[%lu, %lf], [[%lf, %lf, %lf], [%lf, %lf, %lf]], [[%lf, %lf, %lf], [%lf, %lf, %lf]]]",
                  &trip_recorder, &sub_meter_trip_recorder, 
		  &current.linear.x, &current.linear.y, &current.linear.z, &current.angular.x, &current.angular.y, &current.angular.z , 
		  &target.linear.x, &target.linear.y, &target.linear.z, &target.angular.x, &target.angular.y, &target.angular.z ) == EOF) return false; 
	return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const AgentState &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, AgentState &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const AgentState& o ) const {
        return trip_recorder == o.trip_recorder && sub_meter_trip_recorder == o.sub_meter_trip_recorder &&
        current == o.current && target == o.target;
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    AgentState& copyFrom ( const T2 &src) {
        trip_recorder = src.trip_recorder;
        sub_meter_trip_recorder = src.sub_meter_trip_recorder;
        current.copyFrom(src.current);
        target.copyFrom(src.target);
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        des.trip_recorder = trip_recorder;
        des.sub_meter_trip_recorder = sub_meter_trip_recorder;
        current.copyTo(des.current);
        target.copyTo(des.target);
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "trip_recorder", trip_recorder );
        ar & make_nvp ( "sub_meter_trip_recorder", sub_meter_trip_recorder );
        ar & make_nvp ( "current", current );
        ar & make_nvp ( "target", target );
    }
};


};
#endif //SHARED_MEM_OBJECTS_AGENT_STATE_H

