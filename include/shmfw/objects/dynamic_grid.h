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
#include <glob.h>
#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {



template <typename T, template<typename...> class Allocator>
class DynamicGrid  {
    typedef boost::interprocess::vector<T,    Allocator<T>    > VectorT;
protected:
    double m_x_min; /// min x in metric units
    double m_x_max; /// max x in metric units
    double m_y_min; /// min y in metric units
    double m_y_max; /// max y in metric units
    double m_resolution;  /// resolution: metric unit = cell * resolution
    size_t m_size_x; /// size x in cells
    size_t m_size_y; /// size y in cells
    VectorT m_map; /// cells
public:

    DynamicGrid ( const Allocator<void>& void_alloc = {} )
        : m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )
        , m_map ( void_alloc ) {
    }

    template<typename T1>
    void copyTo ( T1& des ) const {
        des.setSize ( getXMin(), getXMax(), getYMin(), getYMax(), getResolution() );
        copyDataTo ( des );
    }

    template<typename T1>
    void copyDataTo ( T1& des ) const {
        /// works as long as the data is aligned
	if(size() != des.size()) throw 0;
        memcpy ( &des[0], data(), sizeof ( T ) * size() );
    }

    template<typename T1>
    DynamicGrid& copyFrom ( const T1& src ) {
        setSize ( src.getXMin(), src.getXMax(), src.getYMin(), src.getYMax(), src.getResolution() );
        return copyDataFrom ( src );
    }
    
    template<typename T1>
    DynamicGrid& copyDataFrom ( const T1& src ) {
        /// works as long as the data is aligned
	if(size() != src.size()) throw 0;
        memcpy ( data(), &src[0], sizeof ( T ) * size() );
	return *this;
    }

    /** Changes the size of the grid, ERASING all previous contents.
      * If \a fill_value is left as NULL, the contents of cells may be undefined (some will remain with
      *  their old values, the new ones will have the default cell value, but the location of old values
      *  may change wrt their old places).
      * If \a fill_value is not NULL, it is assured that all cells will have a copy of that value after resizing.
      * \sa resize, fill
      */
    void  setSize (
        const float	x_min, const float x_max,
        const float y_min, const float y_max,
        const float resolution, const T * fill_value = NULL ) {
        // Adjust sizes to adapt them to full sized cells acording to the resolution:
        m_x_min = resolution*round ( x_min/resolution );
        m_y_min = resolution*round ( y_min/resolution );
        m_x_max = resolution*round ( x_max/resolution );
        m_y_max = resolution*round ( y_max/resolution );

        // Res:
        m_resolution = resolution;

        // Now the number of cells should be integers:
        m_size_x = round ( ( m_x_max-m_x_min ) /m_resolution );
        m_size_y = round ( ( m_y_max-m_y_min ) /m_resolution );

        // Cells memory:
        if ( fill_value )  m_map.assign ( m_size_x*m_size_y, *fill_value );
        else m_map.resize ( m_size_x*m_size_y );
    }

    /** Fills all the cells with the same value
      */
    inline void fill ( const T& value ) {
        for ( typename VectorT::iterator it=m_map.begin(); it!=m_map.end(); ++it ) *it=value;
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
        float new_x_min, float new_x_max,
        float new_y_min, float new_y_max,
        const T& defaultValueNewCells, float additionalMarginMeters = 2.0f ) {
        // Is resize really necesary?
        if ( new_x_min>=m_x_min &&
                new_y_min>=m_y_min &&
                new_x_max<=m_x_max &&
                new_y_max<=m_y_max )	return;

        if ( new_x_min>m_x_min ) new_x_min=m_x_min;
        if ( new_x_max<m_x_max ) new_x_max=m_x_max;
        if ( new_y_min>m_y_min ) new_y_min=m_y_min;
        if ( new_y_max<m_y_max ) new_y_max=m_y_max;

        // Additional margin:
        if ( additionalMarginMeters>0 ) {
            if ( new_x_min<m_x_min ) new_x_min= floor ( new_x_min-additionalMarginMeters );
            if ( new_x_max>m_x_max ) new_x_max= ceil ( new_x_max+additionalMarginMeters );
            if ( new_y_min<m_y_min ) new_y_min= floor ( new_y_min-additionalMarginMeters );
            if ( new_y_max>m_y_max ) new_y_max= ceil ( new_y_max+additionalMarginMeters );
        }

        // Adjust sizes to adapt them to full sized cells acording to the resolution:
        if ( fabs ( new_x_min/m_resolution - round ( new_x_min/m_resolution ) ) >0.05f )
            new_x_min = m_resolution*round ( new_x_min/m_resolution );
        if ( fabs ( new_y_min/m_resolution - round ( new_y_min/m_resolution ) ) >0.05f )
            new_y_min = m_resolution*round ( new_y_min/m_resolution );
        if ( fabs ( new_x_max/m_resolution - round ( new_x_max/m_resolution ) ) >0.05f )
            new_x_max = m_resolution*round ( new_x_max/m_resolution );
        if ( fabs ( new_y_max/m_resolution - round ( new_y_max/m_resolution ) ) >0.05f )
            new_y_max = m_resolution*round ( new_y_max/m_resolution );

        // Change the map size: Extensions at each side:
        unsigned int extra_x_izq = round ( ( m_x_min-new_x_min ) / m_resolution );
        unsigned int extra_y_arr = round ( ( m_y_min-new_y_min ) / m_resolution );

        unsigned int new_size_x = round ( ( new_x_max-new_x_min ) / m_resolution );
        unsigned int new_size_y = round ( ( new_y_max-new_y_min ) / m_resolution );

        // Reserve new memory:
        VectorT new_map(m_map.get_allocator());
        new_map.resize ( new_size_x*new_size_y,defaultValueNewCells );

        // Copy previous rows:
        unsigned int x,y;
        typename VectorT::iterator itSrc;
	typename VectorT::iterator itDst;
        for ( y=0; y<m_size_y; y++ ) {
            for ( x=0,itSrc= ( m_map.begin() +y*m_size_x ),itDst= ( new_map.begin() +extra_x_izq + ( y+extra_y_arr ) *new_size_x );
                    x<m_size_x;
                    x++,itSrc++,itDst++ ) {
                *itDst = *itSrc;
            }
        }

        // Update the new map limits:
        m_x_min = new_x_min;
        m_x_max = new_x_max;
        m_y_min = new_y_min;
        m_y_max = new_y_max;

        m_size_x = new_size_x;
        m_size_y = new_size_y;

        // Keep the new map only:
        m_map.swap(new_map);
	//copyDataFrom( new_map );
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
    inline double getXMin() const  {
        return m_x_min;
    }

    /** Returns the "x" coordinate of right side of grid map.
        */
    inline double getXMax() const  {
        return m_x_max;
    }

    /** Returns the "y" coordinate of top side of grid map.
        */
    inline double getYMin() const  {
        return m_y_min;
    }

    /** Returns the "y" coordinate of bottom side of grid map.
        */
    inline double getYMax() const  {
        return m_y_max;
    }

    /** Returns the resolution of the grid map.
        */
    inline double getResolution() const  {
        return m_resolution;
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& fastCellByIndex ( int idx )  {
        return m_map[idx];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline const T& fastCellByIndex ( int idx ) const {
        return m_map[idx];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& fastCellByIndex ( int cx, int cy ) {        
        return m_map[ cx + cy*this->m_size_x ];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline const T& fastCellByIndex ( int cx, int cy ) const {     
        return m_map[ cx + cy*this->m_size_x ];
    };

    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline T*	cellByPos ( double x, double y ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return NULL;

        return &fastCellByIndex ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline const T* cellByPos ( double x, double y ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return NULL;

        return &fastCellByIndex ( cx, cy );
    }
    /** set the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void setCellByPos ( double x, double y, const T &src ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return;

        fastCellByIndex ( cx, cy ) = src;
    }
    /** Gets the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void getCellByPos ( double x, double y, T &des ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return;

        des = fastCellByIndex ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline  T*	cellByIndex ( unsigned int cx, unsigned int cy ) {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return NULL;
        else	return &fastCellByIndex ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline const T* cellByIndex ( unsigned int cx, unsigned int cy ) const {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return NULL;
        else	return &fastCellByIndex ( cx, cy );
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void setCellByIndex ( unsigned int cx, unsigned int cy, const T &src ) {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return;
        else
            fastCellByIndex ( cx, cy ) = src;
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void getCellByIndex ( unsigned int cx, unsigned int cy, T &des ) const {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return;
        else
            des = fastCellByIndex ( cx, cy );
    }
    /** Transform a coordinate values into cell indexes.
      */
    inline int   x2idx ( double x ) const {
        return static_cast<int> ( ( x-this->m_x_min ) /this->m_resolution );
    }
    inline int   y2idx ( double y ) const {
        return static_cast<int> ( ( y-this->m_y_min ) /this->m_resolution );
    }
    inline int   xy2idx ( double x,double y ) const {
        return x2idx ( x ) + y2idx ( y ) *this->m_size_x;
    }

    /** Transform a global (linear) cell index value into its corresponding (x,y) cell indexes. */
    inline void  idx2cxcy ( const int &idx,  int &cx, int &cy ) const {
        cx = idx % this->m_size_x;
        cy = idx / this->m_size_x;
    }

    /** Transform a cell index into a coordinate value.
      */
    inline double   idx2x ( int cx ) const {
        return this->m_x_min+ ( cx+0.5f ) *this->m_resolution;
    }
    inline double   idx2y ( int cy ) const {
        return this->m_y_min+ ( cy+0.5f ) *this->m_resolution;
    }

    /** Transform a coordinate value into a cell index, using a diferent "x_min" value
        */
    inline int   x2idx ( double x,double x_min ) const {
        SHMFW_UNUSED_PARAM ( x_min );
        return static_cast<int> ( ( x-this->m_x_min ) / this->m_resolution );
    }
    inline int   y2idx ( double y, double y_min ) const {
        SHMFW_UNUSED_PARAM ( y_min );
        return static_cast<int> ( ( y-this->m_y_min ) /this->m_resolution );
    }

    /** The user must implement this in order to provide "saveToTextFile" a way to convert each cell into a numeric value */
    double cell2double ( const T& c ) const {
        SHMFW_UNUSED_PARAM ( c );
        return 0;
    }
    friend std::ostream& operator<< ( std::ostream &output, const DynamicGrid &o ) {
        char msg[0xFF];
        sprintf ( msg, "%zu %zu @ %4.3fm/p of %zu bytes, range x:  %4.3f -> %4.3f, y: %4.3f -> %4.3f",
                  o.getSizeX(), o.getSizeY(), o.getResolution(),
                  sizeof ( T ),
                  o.getXMin(), o.getXMax(),
                  o.getYMin(), o.getYMax() );
        output << msg;
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, DynamicGrid &o ) {
        return input;
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    const T &operator[] ( int idx ) const {
        return fastCellByIndex ( idx );
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    T &operator[] ( int idx ) {
        return fastCellByIndex ( idx );
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    const T &operator() ( int cx, int cy ) const {
        return fastCellByIndex ( cx, cy );
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    T &operator() ( int cx, int cy ) {
        return fastCellByIndex ( cx, cy );
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    const T *data() const {
        return &m_map[0];
    }
    /** Returns a reference ot a cell, no boundary checks are performed.
      */
    T *data() {
        return &m_map[0];
    }
    /** Returns the number of cells.
      */
    size_t size() const{
        return m_map.size();
    }
};

template <typename T>
using AllocatorShm = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

// Variant to use on the heap:
using DynamicGridInt8Heap  = DynamicGrid<int8_t, std::allocator>;
// Variant to use in shared memory:
using DynamicGridInt8Shm = DynamicGrid<int8_t, AllocatorShm>;

// Variant to use on the heap:
using DynamicGrid64FHeap  = DynamicGrid<double, std::allocator>;
// Variant to use in shared memory:
using DynamicGrid64FShm = DynamicGrid<double, AllocatorShm>;

};


#endif //SHARED_MEM_OBJECT_DYNAMIC_GRID_H





