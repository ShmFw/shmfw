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

#ifndef SHARED_MEM_ROS_HEADER_H
#define SHARED_MEM_ROS_HEADER_H


#include <string>
#include <sstream>
#include <iostream>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <shmfw/objects/point.h>
#include <shmfw/header.h>
#include <shmfw/string.h>

#ifdef INCLUDE_ROS_HEADERS  
  #include <ros/ros.h>
#endif
  
namespace ShmFw {
namespace ros {


class Header {
public:
    uint32_t seq;
    boost::posix_time::ptime stamp;
    CharString frame_id;

    Header ( const VoidAllocator &void_alloc )
        : seq ( 0 )
        , stamp( boost::posix_time::microsec_clock::local_time())
        , frame_id ( void_alloc ) {
    }
    Header ( const Header &o, const VoidAllocator &void_alloc )
        : seq ( o.seq )
        , stamp ( o.stamp )
        , frame_id (o.frame_id, void_alloc ) {
    }

    std::string getToString() const {
        std::stringstream ss;
        ss << "[ ";
        ss << frame_id << ", ";
        ss << seq << ", ";
        ss << to_simple_string(stamp) << "]";
        return ss.str();
    }
    void getFromString ( const std::string &str ) {
    }
    friend std::ostream& operator<< ( std::ostream &output, const Header &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Header &o ) {
        return input;
    }
    bool operator == ( const Header& o ) const {
        if ( frame_id.compare ( o.frame_id ) != 0 ) return false;
        if ( seq != o.seq ) return false;
        if ( stamp != o.stamp ) return false;
        return true;
    }
    Header &operator = ( const Header& o ) {
        copyFrom ( o );
        return *this;
    }
    void copyTo ( Header& des ) const {
        des.frame_id = frame_id.c_str();
        des.seq = seq;
        des.stamp = stamp;
    }
    void clear ( ){
        frame_id.clear();
        seq = 0;
        stamp = boost::posix_time::microsec_clock::local_time();
    }
    Header& copyFrom ( const Header& src ) {
        frame_id = src.frame_id.c_str();
        seq = src.seq;
        stamp = src.stamp;
        return *this;
    }
#ifdef ROSCPP_ROS_H  
    void copyTo ( std_msgs::Header& des ) const {
        des.frame_id = this->frame_id.c_str();
        des.seq = this->seq;
        des.stamp = des.stamp.fromBoost(stamp);
    }
    void copyFrom (const std_msgs::Header& src )  {
        this->frame_id = src.frame_id.c_str();
        this->seq = src.seq;
        this->stamp = src.stamp.toBoost();
    }
#endif
};
};
};


#endif //SHARED_MEM_ROS_HEADER_H


