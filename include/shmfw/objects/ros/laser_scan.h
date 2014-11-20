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

#ifndef SHARED_MEM_ROS_LASER_SCAN
#define SHARED_MEM_ROS_LASER_SCAN

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>

namespace bi = boost::interprocess;

namespace ShmFw {
namespace ros {

/**
 * This class is used for laser range scanners
 * @note this class can only be used in combination with ShmFw::Alloc 
 **/ 
class LaserScan {
    typedef bi::managed_shared_memory::segment_manager SegmentManager;
    typedef bi::allocator<void,   SegmentManager> AllocatorVoid;
    typedef bi::allocator<char,   SegmentManager> AllocatorChar;
    typedef bi::allocator<double, SegmentManager> AllocatorDouble;
    typedef bi::basic_string<char, std::char_traits<char>, AllocatorChar>   CharString;
    typedef bi::vector<double, AllocatorDouble > VectorDouble;
public:
    CharString frame;
    double angle_min;        // start angle of the scan [rad]
    double angle_max;        // end angle of the scan [rad]
    double angle_increment;  // angular distance between measurements [rad]

    double time_increment;   // time between measurements [seconds] - if your scanner
    // is moving, this will be used in interpolating position of 3d points
    double scan_time;        // time between scans [seconds]

    double range_min;        // minimum range value [m]
    double range_max;        // maximum range value [m]
    VectorDouble ranges;     // range data [m] (Note: values < range_min or > range_max should be discarded)
    VectorDouble intensities;// intensity data [device-specific units].  If your
    // device does not provide intensities, please leave the array empty.
    LaserScan ( const AllocatorVoid &void_alloc )
        : frame ( void_alloc ), range_min ( 0 ), range_max ( 0 ), ranges ( void_alloc ), intensities ( void_alloc )
    {}

    std::string getToString() const {
        char header[0xFF];
        sprintf ( header, "%s: %frad <-> %frad\n", frame.c_str(), range_min, range_max );
        std::stringstream ss ( header );
        for ( size_t i = 0; i < ranges.size(); i++ ) {
            ss << ranges[i] << ( ( i != ranges.size()-1 ) ?", ":";\n" );
        }
        return ss.str();
    }
    friend std::ostream& operator<< ( std::ostream &output, const LaserScan &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, LaserScan &o ) {
        return input;
    }
};
};
};


#endif //SHARED_MEM_ROS_LASER_SCAN


