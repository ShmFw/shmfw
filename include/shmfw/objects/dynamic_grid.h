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
#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {

  

template <typename T> class DynamicGrid {
protected:
    double m_x_min; /// min x in metric units
    double m_x_max; /// max x in metric units
    double m_y_min; /// min y in metric units
    double m_y_max; /// max y in metric units
    double m_resolution;  /// resolution: metric unit = cell * resolution
    size_t m_size_x; /// size x in cells
    size_t m_size_y; /// size y in cells
    size_t m_size;   /// size total number of cells = m_size_x*m_size_y
    boost::interprocess::offset_ptr<T> m_data;      /// data;
public:
    DynamicGrid()
        : m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )
        , m_size ( 0 )
        , m_data ( NULL )   {
    }
    DynamicGrid ( const DynamicGrid &src )
        : m_x_min ( src.m_x_min )
        , m_x_max ( src.m_x_max )
        , m_y_min ( src.m_y_min )
        , m_y_max ( src.m_y_max )
        , m_resolution ( src.m_resolution )
        , m_size_x ( src.m_size_x )
        , m_size_y ( src.m_size_y )
        , m_size ( src.m_size_x * src.m_size_y )
        , m_data ( src.m_data )  {
    }

    virtual void resize ( std::size_t n ) = 0;
    virtual void assign ( std::size_t n, const T& val ) = 0;
    virtual void setSize (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double resolution, const T * fill_value = NULL ) = 0;

    template<typename T1>
    void copyTo ( T1& des, bool use_memcpy = true ) const {
        des.m_x_min = m_x_min;
        des.m_x_max = m_x_max;
        des.m_y_min = m_y_min;
        des.m_y_max = m_y_max;
        des.m_resolution = m_resolution;
        des.m_size_x = m_size_x;
        des.m_size_y = m_size_y;
        des.resize ( m_size_x * m_size_y );
        copyDataTo ( des, use_memcpy );
    }

    template<typename T1>
    void copyDataTo ( T1& des, bool use_memcpy = true ) const {
	if(use_memcpy) 
	  memcpy( &des[0], &m_data[0], sizeof(T) * size());
        else 
	  for ( size_t i = 0; i < size(); i++ ) des[i] = m_data[i];
    }

    template<typename T1>
    DynamicGrid& copyFrom ( const T1& src, bool use_memcpy = true ) {
        m_x_min = src.m_x_min;
        m_x_max = src.m_x_max;
        m_y_min = src.m_y_min;
        m_y_max = src.m_y_max;
        m_resolution = src.m_resolution;
        m_size_x = src.m_size_x;
        m_size_y = src.m_size_y;
        resize ( m_size_x*m_size_y );
        return copyDataFrom ( src, use_memcpy );
    }
    template<typename T1>
    DynamicGrid& copyDataFrom ( const T1& src, bool use_memcpy = true ) {
	if (use_memcpy) 
	  memcpy( &m_data[0], &src[0], sizeof(T) * size());
        else 
	  for ( size_t i = 0; i < size(); i++ )  m_data[i] = src[i];  
        return *this;
    }

    /** Returns the number of cells.
        */
    inline size_t size() const {
        return m_size;
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
    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline T*	cellByPos ( float x, float y ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return NULL;

        return &this->m_data[ cx + cy*this->m_size_x ];
    }

    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline const T* cellByPos ( float x, float y ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return NULL;

        return &this->m_data[ cx + cy*this->m_size_x ];
    }
    /** set the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void setCellByPos ( float x, float y, const T &src ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return;

        this->m_data[ cx + cy*this->m_size_x ] = src;
    }
    /** Gets the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void getCellByPos ( float x, float y, T &des ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return;

        des = this->m_data[ cx + cy*this->m_size_x ];
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline  T*	cellByIndex ( unsigned int cx, unsigned int cy ) {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return NULL;
        else	return &this->m_data[ cx + cy*this->m_size_x ];
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline const T* cellByIndex ( unsigned int cx, unsigned int cy ) const {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return NULL;
        else	return &this->m_data[ cx + cy*this->m_size_x ];
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void setCellByIndex ( unsigned int cx, unsigned int cy, const T &src ) {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return;
        else
            this->m_data[ cx + cy*this->m_size_x ] = src;
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void getCellByIndex ( unsigned int cx, unsigned int cy, T &des ) const {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return;
        else
            des = this->m_data[ cx + cy*this->m_size_x ];
    }
    /** Transform a coordinate values into cell indexes.
      */
    inline int   x2idx ( float x ) const {
        return static_cast<int> ( ( x-this->m_x_min ) /this->m_resolution );
    }
    inline int   y2idx ( float y ) const {
        return static_cast<int> ( ( y-this->m_y_min ) /this->m_resolution );
    }
    inline int   xy2idx ( float x,float y ) const {
        return x2idx ( x ) + y2idx ( y ) *this->m_size_x;
    }

    /** Transform a global (linear) cell index value into its corresponding (x,y) cell indexes. */
    inline void  idx2cxcy ( const int &idx,  int &cx, int &cy ) const {
        cx = idx % this->m_size_x;
        cy = idx / this->m_size_x;
    }

    /** Transform a cell index into a coordinate value.
      */
    inline float   idx2x ( int cx ) const {
        return this->m_x_min+ ( cx+0.5f ) *this->m_resolution;
    }
    inline float   idx2y ( int cy ) const {
        return this->m_y_min+ ( cy+0.5f ) *this->m_resolution;
    }

    /** Transform a coordinate value into a cell index, using a diferent "x_min" value
        */
    inline int   x2idx ( float x,float x_min ) const {
        SHMFW_UNUSED_PARAM ( x_min );
        return static_cast<int> ( ( x-this->m_x_min ) / this->m_resolution );
    }
    inline int   y2idx ( float y, float y_min ) const {
        SHMFW_UNUSED_PARAM ( y_min );
        return static_cast<int> ( ( y-this->m_y_min ) /this->m_resolution );
    }

    /** The user must implement this in order to provide "saveToTextFile" a way to convert each cell into a numeric value */
    virtual float cell2float ( const T& c ) const {
        SHMFW_UNUSED_PARAM ( c );
        return 0;
    }
    /** Returns a pointer to the raw data.
      */
    inline const T *raw () const {
        return m_data;
    }
    /** Returns a pointer to the raw data.
      */
    inline T *raw () {
        return m_data;
    }
    friend std::ostream& operator<< ( std::ostream &output, const DynamicGrid &o ) {
        char msg[0xFF];
        sprintf ( msg, "%zu %zu @ %4.3fm/p of %zu bytes, range x:  %4.3f -> %4.3f, y: %4.3f -> %4.3f",
                  o.m_size_x, o.m_size_y, o.m_resolution,
                  sizeof ( T ),
                  o.m_x_min, o.m_x_max,
                  o.m_y_min, o.m_y_max );
        output << msg;
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, DynamicGrid &o ) {
        return input;
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    inline const T &operator[] ( int idx ) const {
        return m_data[idx];
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    inline T &operator[] ( int idx ) {
        return m_data[idx];
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    inline const T &operator() ( unsigned int cx, unsigned int cy ) const {
        return this->m_data[ cx + cy*this->m_size_x ];
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    inline T &operator() ( unsigned int cx, unsigned int cy ) {
        return this->m_data[ cx + cy*this->m_size_x ];
    }
};

template <typename T, template<typename...> class Allocator>
class DynamicGridData : public DynamicGrid<T> {
    typedef boost::interprocess::vector<T,    Allocator<T>    > VectorT;
public:
    VectorT m_map; /// cells
public:
    DynamicGridData ( const Allocator<void>& void_alloc = {} )
        : DynamicGrid<T>()
        , m_map ( void_alloc ) {
    }

    DynamicGridData ( const DynamicGridData &src )
        : DynamicGrid<T> ( src )
        , m_map ( src.m_map.get_allocator() ) {
        this->copyDataFrom ( src );
    }

    virtual void  setSize (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double resolution, const T * fill_value = NULL ) {
        // Adjust sizes to adapt them to full sized cells acording to the resolution:
        this->m_x_min = resolution*round ( x_min/resolution );
        this->m_y_min = resolution*round ( y_min/resolution );
        this->m_x_max = resolution*round ( x_max/resolution );
        this->m_y_max = resolution*round ( y_max/resolution );

        // Res:
        this->m_resolution = resolution;

        // Now the number of cells should be integers:
        this->m_size_x = round ( ( this->m_x_max-this->m_x_min ) / this->m_resolution );
        this->m_size_y = round ( ( this->m_y_max-this->m_y_min ) / this->m_resolution );

        // Cells memory:
        if ( fill_value )
            assign ( this->m_size_x * this->m_size_y, *fill_value );
        else
            resize ( this->m_size_x * this->m_size_y );
    }

    virtual void resize ( std::size_t n ) {
        this->m_size = n;
        m_map.resize ( this->m_size );
        this->m_data = &m_map[0];
    }
    virtual void assign ( std::size_t n, const T& val ) {
        this->m_size = n;
        m_map.assign ( this->m_size, val );
        this->m_data = &m_map[0];
    }
    static boost::shared_ptr<DynamicGridData<T, Allocator> > create(){
      return boost::shared_ptr<DynamicGridData<T, Allocator> >(new (DynamicGridData<T, Allocator> ));
    }
};


template <typename T>
using AllocatorShm = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

// Variant to use on the heap:
using DynamicGridInt8Heap  = DynamicGridData<int8_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridInt8Shm = DynamicGridData<int8_t, AllocatorShm>;

// Variant to use on the heap:
using DynamicGrid64FHeap  = DynamicGridData<double, std::allocator>;
// Variant to use in shared memory:
using DynamicGrid64FShm = DynamicGridData<double, AllocatorShm>;

};


#endif //SHARED_MEM_OBJECT_DYNAMIC_GRID_H


