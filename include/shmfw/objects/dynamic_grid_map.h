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

#ifndef SHARED_MEM_OBJECT_DYNAMIC_GRID_MAP_H
#define SHARED_MEM_OBJECT_DYNAMIC_GRID_MAP_H

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <glob.h>
#include <shmfw/handler.h>

#include <shmfw/objects/grid_map.h>

#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {
  
template <typename T, template<typename...> class Allocator>
class DynamicGridMap : public GridMap<T> {
    typedef boost::interprocess::vector<T,    Allocator<T>    > VectorT;
protected:
    VectorT m_map; /// cells
public:

    DynamicGridMap ( const Allocator<void>& void_alloc = {} )
        : GridMap<T> ()
        , m_map ( void_alloc ) {
    }

    template<typename T1>
    void copyTo ( T1& des ) const {
        des.setSizeWithResolution ( this->getXMin(), this->getXMax(), this->getYMin(), this->getYMax(), this->getResolutionX(), this->getResolutionY() );
        this->copyDataTo ( des );
    }


    template<typename T1>
    DynamicGridMap& copyFrom ( const T1& src ) {
        setSizeWithResolution ( src.getXMin(), src.getXMax(), src.getYMin(), src.getYMax(), src.getResolutionX(), src.getResolutionY()  );
        return this->copyDataFrom ( src );
    }

    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithResolution (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double x_resolution, const double y_resolution) {

        // Sets the bounderies and rounds the values to integers if needed
        this->setBounderies ( x_min, x_max, y_min, y_max, x_resolution, y_resolution );

        // Cells memory:
        m_map.resize ( this->m_size_x*this->m_size_y );
        this->m_data = &m_map[0];
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithResolution (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double x_resolution, const double y_resolution, const T &fill_value ) {

        // Sets the bounderies and rounds the values to integers if needed
        this->setBounderies ( x_min, x_max, y_min, y_max, x_resolution, y_resolution );

        m_map.assign ( this->m_size_x*this->m_size_y, fill_value );
        this->m_data = &m_map[0];
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithNrOfCells (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const size_t size_x, const size_t size_y ) {

        // Sets the bounderies and rounds the values to integers if needed
        this->setBounderies ( x_min, x_max, y_min, y_max, size_x,  size_y);

        // Cells memory:
        m_map.resize ( this->m_size_x*this->m_size_y );
        this->m_data = &m_map[0];
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithNrOfCells (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const size_t size_x, const size_t size_y, const T &fill_value  ) {

        // Sets the bounderies and rounds the values to integers if needed
        this->setBounderies ( x_min, x_max, y_min, y_max, size_x,  size_y);

        // Cells memory:
        m_map.assign ( this->m_size_x*this->m_size_y, fill_value );
        this->m_data = &m_map[0];
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSize (const GridMap<T> &map, bool copy = false){

        // Sets the bounderies and rounds the values to integers if needed
        this->setBounderies ( map.getXMin(), map.getXMax(), map.getYMin(), map.getYMax(), map.getSizeX(),  map.getSizeY());

        // Cells memory:
        m_map.resize ( this->m_size_x*this->m_size_y);
        this->m_data = &m_map[0];
	if(copy){
	  this->copyDataFrom(map);
	}
    }


    /** Erase the contents of all the cells. */
    void  clear() {
        m_map.clear();
        m_map.resize ( this->m_size_x*this->m_size_y );
    }

    /** Changes the size of the grid, maintaining previous contents.
      * \sa setSize
      */
    void  resize (
        double new_x_min, double new_x_max,
        double new_y_min, double new_y_max,
        const T& defaultValueNewCells, double additionalMarginMeters = 2.0f ) {
        // Is resize really necesary?
        if ( new_x_min >= this->m_x_min &&
                new_y_min >= this->m_y_min &&
                new_x_max <= this->m_x_max &&
                new_y_max <= this->m_y_max )	return;

        if ( new_x_min > this->m_x_min ) new_x_min = this->m_x_min;
        if ( new_x_max < this->m_x_max ) new_x_max = this->m_x_max;
        if ( new_y_min > this->m_y_min ) new_y_min = this->m_y_min;
        if ( new_y_max < this->m_y_max ) new_y_max = this->m_y_max;

        // Additional margin:
        if ( additionalMarginMeters>0 ) {
            if ( new_x_min < this->m_x_min ) new_x_min = floor ( new_x_min-additionalMarginMeters );
            if ( new_x_max > this->m_x_max ) new_x_max = ceil ( new_x_max+additionalMarginMeters );
            if ( new_y_min < this->m_y_min ) new_y_min = floor ( new_y_min-additionalMarginMeters );
            if ( new_y_max > this->m_y_max ) new_y_max = ceil ( new_y_max+additionalMarginMeters );
        }

        // Adjust sizes to adapt them to full sized cells acording to the resolution:
        if ( fabs ( new_x_min/this->m_x_resolution - round ( new_x_min/this->m_x_resolution ) ) >0.05f )
            new_x_min = this->m_x_resolution*round ( new_x_min/this->m_x_resolution );
        if ( fabs ( new_y_min/this->m_y_resolution - round ( new_y_min/this->m_y_resolution ) ) >0.05f )
            new_y_min = this->m_y_resolution*round ( new_y_min/this->m_y_resolution );
        if ( fabs ( new_x_max/this->m_x_resolution - round ( new_x_max/this->m_x_resolution ) ) >0.05f )
            new_x_max = this->m_x_resolution*round ( new_x_max/this->m_x_resolution );
        if ( fabs ( new_y_max/this->m_y_resolution - round ( new_y_max/this->m_y_resolution ) ) >0.05f )
            new_y_max = this->m_y_resolution*round ( new_y_max/this->m_y_resolution );

        // Change the map size: Extensions at each side:
        unsigned int extra_x_izq = round ( ( this->m_x_min-new_x_min ) / this->m_x_resolution );
        unsigned int extra_y_arr = round ( ( this->m_y_min-new_y_min ) / this->m_y_resolution );

        unsigned int new_size_x = round ( ( new_x_max-new_x_min ) / this->m_x_resolution );
        unsigned int new_size_y = round ( ( new_y_max-new_y_min ) / this->m_y_resolution );

        // Reserve new memory:
        VectorT new_map ( m_map.get_allocator() );
        new_map.resize ( this->new_size_x*this->new_size_y,defaultValueNewCells );

        // Copy previous rows:
        unsigned int x,y;
        typename VectorT::iterator itSrc;
        typename VectorT::iterator itDst;
        for ( y = 0; y < this->m_size_y; y++ ) {
            for ( x = 0,itSrc = ( m_map.begin() +y*this->m_size_x ),itDst= ( new_map.begin() +extra_x_izq + ( y+extra_y_arr ) *new_size_x );
                    x<this->m_size_x;
                    x++,itSrc++,itDst++ ) {
                *itDst = *itSrc;
            }
        }

        // Update the new map limits:
        this->m_x_min = new_x_min;
        this->m_x_max = new_x_max;
        this->m_y_min = new_y_min;
        this->m_y_max = new_y_max;

        this->m_size_x = new_size_x;
        this->m_size_y = new_size_y;

        // Keep the new map only:
        m_map.swap ( new_map );
        //copyDataFrom( new_map );
        this->m_data = &m_map[0];
    }
    /** Returns the number of cells.
      */
    size_t size() const {
        return m_map.size();
    }
};

// Variant to use on the heap:
using DynamicGridMapS8Heap  = DynamicGridMap<int8_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMapS8Shm = DynamicGridMap<int8_t, Allocator>;
// Variant to use on the heap:
using DynamicGridMapS16Heap  = DynamicGridMap<int16_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMapS16Shm = DynamicGridMap<int16_t, Allocator>;
// Variant to use on the heap:
using DynamicGridMapS32Heap  = DynamicGridMap<int32_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMapS32Shm = DynamicGridMap<int32_t, Allocator>;

// Variant to use on the heap:
using DynamicGridMapU8Heap  = DynamicGridMap<uint8_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMapU8Shm = DynamicGridMap<uint8_t, Allocator>;
// Variant to use on the heap:
using DynamicGridMapU16Heap  = DynamicGridMap<uint16_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMapU16Shm = DynamicGridMap<uint16_t, Allocator>;
// Variant to use on the heap:
using DynamicGridMapU32Heap  = DynamicGridMap<uint32_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMapU32Shm = DynamicGridMap<uint32_t, Allocator>;

// Variant to use on the heap:
using DynamicGridMap64FHeap  = DynamicGridMap<double, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMap64FShm = DynamicGridMap<double, Allocator>;
// Variant to use on the heap:
using DynamicGridMap32FHeap  = DynamicGridMap<float, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMap32FShm = DynamicGridMap<float, Allocator>;
// Variant to use on the heap:
using DynamicGridMap8UHeap  = DynamicGridMap<uchar, std::allocator>;
// Variant to use in shared memory:
using DynamicGridMap8UShm = DynamicGridMap<uchar, Allocator>;
// Variant to use on the heap:
using DynamicGridMap8U3CHeap  = DynamicGridMap<uchar[3], std::allocator>;
// Variant to use in shared memory:
using DynamicGridMap8U3CShm = DynamicGridMap<uchar[3], Allocator>;

};


#endif //SHARED_MEM_OBJECT_DYNAMIC_GRID_MAP_H





