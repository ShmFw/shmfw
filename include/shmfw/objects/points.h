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

#ifndef SHARED_MEM_CLASSES_POINTS_H
#define SHARED_MEM_CLASSES_POINTS_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <shmfw/objects/point.h>
#include <shmfw/handler.h>


namespace ShmFw {

/**
 * @note this class can only be used in combination with ShmFw::Alloc
 **/
namespace bi = boost::interprocess;
class Points {
    typedef bi::managed_shared_memory::segment_manager SegmentManager;
    typedef bi::allocator<void,   SegmentManager> AllocatorVoid;
    typedef bi::allocator<ShmFw::Point, SegmentManager> AllocatorPoint;
    typedef bi::vector<ShmFw::Point, AllocatorPoint > VectorPoints;

public:
    CharString frame;
    VectorPoints points;

    Points ( const AllocatorVoid &void_alloc )
        : frame ( void_alloc ), points ( void_alloc )
    {}

    Points ( const Points &p )
        : frame ( p.frame.get_allocator() ), points ( p.points.get_allocator() ) {
        copyFrom ( p );
    }

    std::string getToString() const {
        std::stringstream ss;
        ss << frame << ", ";
        for ( size_t i = 0; i < points.size(); i++ ) {
            ss << points[i] << ( ( i != points.size()-1 ) ?", ":";\n" );
        }
        return ss.str();
    }
    void getFromString ( const std::string &str ) {
    }
    friend std::ostream& operator<< ( std::ostream &output, const Points &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Points &o ) {
        return input;
    }
    bool operator == ( const Points& o ) const {
        if ( points.size() != o.points.size() ) return false;
        if ( frame.compare ( o.frame ) != 0 ) return false;
        for ( VectorPoints::const_iterator it0 = points.begin(),  it1 = o.points.begin(); it1 != o.points.end(); it1++, it0++ ) {
            if ( !(*it0 == *it1) ) return false;
        }
        return true;
    }
    Points &operator = ( const Points& o ) {
        copyFrom(o);
        return *this;
    }
    template<typename T>
    void copyTo ( T& des ) const {
        des.frame = frame;
        des.points = points;
    }
    template<typename T>
    Points& copyFrom ( const T& src ) {
        frame = src.frame;
        points = src.points;
        return *this;
    }
};

};


#endif //SHARED_MEM_CLASSES_POINTS_H


