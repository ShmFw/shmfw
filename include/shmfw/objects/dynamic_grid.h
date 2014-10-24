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

#ifndef SHARED_MEM_OBJECT_DYNAMIC_GRID_H
#define SHARED_MEM_OBJECT_DYNAMIC_GRID_H

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace ShmFw {
template <template<typename...> class Allocator>
class DynamicGrid {
    typedef boost::interprocess::vector<int8_t,    Allocator<int8_t>    > VectorT;

    /** The cells.
      */
    VectorT m_map; /// cells
    float m_x_min; /// min x in metric units
    float m_x_max; /// max x in metric units
    float m_y_min; /// min y in metric units
    float m_y_max; /// max y in metric units
    float m_resolution;  /// resolution: metric unit = cell * resolution
    size_t m_size_x; /// size x in cells
    size_t m_size_y; /// size y in cells
public:

    DynamicGrid ( const Allocator<void>& void_alloc = {} )
        : m_map ( void_alloc )
        , m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )    {

    }

    DynamicGrid ( const DynamicGrid &p )
        : m_map ( p.m_map.get_allocator() )
        , m_x_min ( p.m_x_min )
        , m_x_max ( p.m_x_max )
        , m_y_min ( p.m_y_min )
        , m_y_max ( p.m_y_max )
        , m_resolution ( p.m_resolution )
        , m_size_x ( p.m_size_x )
        , m_size_y ( p.m_size_y ) {

    }

    template<typename T2>
    void copyTo ( T2& des ) const {
        des.m_x_min = m_x_min;
        des.m_x_max = m_x_max;
        des.m_y_min = m_y_min;
        des.m_y_max = m_y_max;
        des.m_resolution = m_resolution;
        des.m_size_x = m_size_x;
        des.m_size_y = m_size_y;
        des.m_map.resize ( m_map.size() );
        for ( size_t i = 0; i < des.m_map.size(); i++ ) {
            des.data[i] = m_map[i];
        }
    }
    template<typename T2>
    DynamicGrid& copyFrom ( const T2& src ) {
        m_x_min = src.m_x_min;
        m_x_max = src.m_x_max;
        m_y_min = src.m_y_min;
        m_y_max = src.m_y_max;
        m_resolution = src.m_resolution;
        m_size_x = src.m_size_x;
        m_size_y = src.m_size_y;
        m_map.resize ( src.m_map.size() );
        for ( size_t i = 0; i < src.m_map.size(); i++ ) {
            m_map[i] = src.m_map[i];
        }
        return *this;
    }


    /** Returns the horizontal size of grid map in cells count.
        */
    inline size_t getSizeX() const {
        return m_size_x;
    }

    /** Returns the vertical size of grid map in cells count.
        */
    inline size_t getSizeY() const {
        return m_size_y;
    }

    /** Returns the "x" coordinate of left side of grid map.
        */
    inline float  getXMin() const  {
        return m_x_min;
    }

    /** Returns the "x" coordinate of right side of grid map.
        */
    inline float  getXMax() const  {
        return m_x_max;
    }

    /** Returns the "y" coordinate of top side of grid map.
        */
    inline float  getYMin() const  {
        return m_y_min;
    }

    /** Returns the "y" coordinate of bottom side of grid map.
        */
    inline float  getYMax() const  {
        return m_y_max;
    }

    /** Returns the resolution of the grid map.
        */
    inline float  getResolution() const  {
        return m_resolution;
    }

    friend std::ostream& operator<< ( std::ostream &output, const DynamicGrid<Allocator> &o ) {
        char msg[0xFF];
        sprintf (msg, "%zu %zu @ %4.3fm/p of %s, %4.3f, %4.3f, %4.3f, %4.3f\n",
                  o.getSizeX(), o.getSizeY(), o.getResolution(),
                  typeid ( uint8_t ).name(),
                  o.getXMin(), o.getYMin(),
                  o.getXMax(), o.getYMax() );
        output << msg;
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, DynamicGrid<Allocator> &o ) {
        return input;
    }
};

template <typename T>
using AllocatorShm = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

// Variant to use on the heap:
using DynamicGridHeap  = DynamicGrid<std::allocator>;
// Variant to use in shared memory:
using DynamicGridShm = DynamicGrid<AllocatorShm>;


};


#endif //SHARED_MEM_OBJECT_DYNAMIC_GRID_H


