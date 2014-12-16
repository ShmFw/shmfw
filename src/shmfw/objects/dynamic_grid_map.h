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
#include <shmfw/header.h>

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
        des.setSizeWithResolution ( this->getXMin(), this->getXMax(), this->getYMin(), this->getYMax(), this->getResolutionX(), this->getResolutionY(), 1 );
        this->copyDataTo ( des );
    }


    template<typename T1>
    DynamicGridMap& copyFrom ( const T1& src ) {
        setSizeWithResolution ( src.getXMin(), src.getXMax(), src.getYMin(), src.getYMax(), src.getResolutionX(), src.getResolutionY(),1  );
        return this->copyDataFrom ( src );
    }

    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithResolution (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double x_resolution, const double y_resolution,
	const size_t layers) {

        // Sets the bounderies and rounds the values to integers if needed
        this->initHeader ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, sizeof(T), layers );

        // Cells memory:
        m_map.resize ( this->size_total());
        this->m_origin_data = &m_map[0];
	this->activateLayer(0);
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithResolution (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double x_resolution, const double y_resolution,
	const size_t layers, const T &fill_value ) {

        // Sets the bounderies and rounds the values to integers if needed
        this->initHeader ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, sizeof(T), layers);

        m_map.assign ( this->size_total(), fill_value );
        this->m_origin_data = &m_map[0];
	this->activateLayer(0);
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithNrOfCells (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const size_t size_x, const size_t size_y,
	const size_t layers ) {

        // Sets the bounderies and rounds the values to integers if needed
        this->initHeader ( x_min, x_max, y_min, y_max, size_x,  size_y, sizeof(T), layers);

        // Cells memory:
        m_map.resize ( this->size_total() );
        this->m_origin_data = &m_map[0];
	this->activateLayer(0);
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    void  setSizeWithNrOfCells (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const size_t size_x, const size_t size_y,
	const size_t layers, const T &fill_value  ) {

        // Sets the bounderies and rounds the values to integers if needed
        this->initHeader ( x_min, x_max, y_min, y_max, size_x,  size_y, sizeof(T), layers);

        // Cells memory:
        m_map.assign (  this->size_total(), fill_value );
        this->m_origin_data = &m_map[0];
	this->activateLayer(0);
    }
    /** Changes the size of the grid, ERASING all previous contents.
      */
    template <typename T1>
    void  setSize (const GridMap<T1> &map, bool copy = false){

        // Sets the bounderies and rounds the values to integers if needed
        this->initHeader ( map.getXMin(), map.getXMax(), map.getYMin(), map.getYMax(), map.getSizeX(),  map.getSizeY(), 
			      map.getDepth(), map.getLayers());

        // Cells memory:
        m_map.resize (this->size_total());
        this->m_origin_data = &m_map[0];
	this->activateLayer(0);
	if(copy){
	  this->copyDataFrom(map);
	}
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

};


#endif //SHARED_MEM_OBJECT_DYNAMIC_GRID_MAP_H





