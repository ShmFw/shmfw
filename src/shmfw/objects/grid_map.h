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

#ifndef SHARED_MEM_OBJECT_GRID_MAP_H
#define SHARED_MEM_OBJECT_GRID_MAP_H

#include <opencv/cxcore.h>
#include <shmfw/objects/grid_map_header.h>
#include <boost/interprocess/offset_ptr.hpp>

#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {


/** A 2D grid of dynamic size which stores any kind of data at each cell.
 * @tparam T The type of each cell in the 2D grid.
 * @note This class is based on the mrpt::slam::CDynamicGridMap which was published unter BSD many thanks to the mrpt team
 */
template <typename T>
class GridMap : public ShmFw::GridMapHeader {
protected:
    boost::interprocess::offset_ptr<T>  m_origin_data; /// cells
    boost::interprocess::offset_ptr<T>  m_data;        /// cells
public:
    GridMap ()
        : GridMapHeader (  )
        , m_data () {
    }
    GridMap ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t layers, T *data, const T * fill_value = NULL ) {
      size_t type_hash_code;
#if __cplusplus > 199711L
        type_hash_code = typeid ( T ).hash_code();
#else
        type_hash_code = 0;
#endif
      initHeader ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, sizeof ( T ), layers, type_hash_code);
      initData(data, fill_value);
    }
    /** Initilialies a given data array
      */
    void initData (T *data, const T * fill_value = NULL ) {
        m_data = data;
        if ( fill_value ) fill ( fill_value );
    }
    /// copies only the active layer to des. @param des destination
    template<typename T1> void copyLayerTo ( T1& des ) const {
        memcpy ( &des[0], data(), bytes() );
    }
    /// copies only the active layer from src. @param src source
    template<typename T1> GridMap& copyLayerFrom ( const T1& src ) {
        memcpy ( data(), &src[0], bytes() );
        return *this;
    }
    /// copies only the active layer from src. @param src source
    void copyLayerFromArray ( const void *src ) {
        memcpy ( data(), src, bytes_total() );
    }
    /// copies all layers layer to des. @param des destination
    template<typename T1> void copyDataTo ( T1& des ) const {
        memcpy ( &des[0], data(), bytes_total() );
    }
    /// copies all layers from src. @param src source
    template<typename T1> GridMap& copyDataFrom ( const T1& src ) {
        memcpy ( origin_data(), &src[0], bytes_total() );
        return *this;
    }
    /// copies all layers from src. @param src source
    void copyDataFromArray ( const void *src ) {
        memcpy ( origin_data(), src, bytes_total() );
    }

    /** Fills all the cells with the same value
      */
    inline void fill ( const T& value ) {
        T *p =  m_data.get();
        T *end = p+size();
        while ( p != end ) {
            *p++ = value;
        }
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    const T &operator[] ( int idx ) const {
        return m_data[idx];
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    T &operator[] ( int idx ) {
        return m_data[idx];
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    const T *data() const {
        return m_data.get();
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    T *data() {
        return m_data.get();
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    T *data(size_t i) {
        return m_data.get() + i;
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    T *origin_data() {
        return m_origin_data.get();
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    const T *origin_data() const {
        return m_origin_data.get();
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    T *origin_data(size_t i) {
        return m_origin_data.get() + i;
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    const T *origin_data(size_t i) const {
        return m_origin_data.get() + i;
    }
    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& cellByIndex_nocheck ( int idx )  {
        return m_data[idx];
    };
    /** Returns a pointer to a layer @return pointer ot layer data
      */
    T* data_layer(size_t layer){
	if(layer >= getLayers()) throw 0;
        return origin_data(size() * layer);
    }
    /** Returns a pointer to a layer @return pointer ot layer data
      */
    const T* data_layer(size_t layer) const {
	if(layer >= getLayers()) throw 0;
        return origin_data(this->size() * layer);
    }
    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline const T& cellByIndex_nocheck ( int idx ) const {
        return m_data[idx];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& cellByIndex_nocheck ( int cx, int cy ) {
        return m_data[ xy2idx(cx,cy) ];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline const T& cellByIndex_nocheck ( int cx, int cy ) const {
        return m_data[ xy2idx(cx,cy) ];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& cellByIndex_nocheck ( const cv::Point &p ) {
        return cellByIndex_nocheck ( p.x, p.y );
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline const T& cellByIndex_nocheck ( const cv::Point &p ) const {
        return cellByIndex_nocheck ( p.x, p.y );
    };

    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline T& cellByPos ( double x, double y ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return NULL;

        return cellByIndex_nocheck ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline const T& cellByPos ( double x, double y ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return NULL;

        return cellByIndex_nocheck ( cx, cy );
    }
    /** set the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void setCellByPos ( double x, double y, const T &src ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return;

        cellByIndex_nocheck ( cx, cy ) = src;
    }
    /** Gets the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void getCellByPos ( double x, double y, T &des ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return;

        des = cellByIndex_nocheck ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline  T*	cellByIndex ( unsigned int cx, unsigned int cy ) {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return NULL;
        else	return &cellByIndex_nocheck ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline const T* cellByIndex ( unsigned int cx, unsigned int cy ) const {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return NULL;
        else	return &cellByIndex_nocheck ( cx, cy );
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void setCellByIndex ( unsigned int cx, unsigned int cy, const T &src ) {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return;
        else
            cellByIndex_nocheck ( cx, cy ) = src;
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void getCellByIndex ( unsigned int cx, unsigned int cy, T &des ) const {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return;
        else
            des = cellByIndex_nocheck ( cx, cy );
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    const T &operator() ( int cx, int cy ) const {
        return cellByIndex_nocheck ( cx, cy );
    }
    /** Returns a reference to a cell, no boundary checks are performed.
      */
    T &operator() ( int cx, int cy ) {
        return cellByIndex_nocheck ( cx, cy );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMatLayer(size_t layer) {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), data_layer(layer) );
    }
    /** Returns the data as opencv matrix
      */
    const cv::Mat_<T> cvMatLayer(size_t layer) const {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), (T*) data_layer(layer) );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMat() {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), m_data.get() );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMat() const {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), m_data.get() );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat cvMat ( int cvtype ) const {
        return cv::Mat ( getSizeY(), getSizeX(), cvtype, m_data.get() );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> &cvMatTotal ( cv::Mat_<T> &m ) {
        m = cvMatTotal();
        return m;
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMatTotal() const {
        return cv::Mat_<T> ( getSizeY()*getLayers(), getSizeX(), (T*) m_origin_data.get() );
    }
    /** creates a opencv line iterator based on a metric start and endpoint
      */
    cv::LineIterator cvLineIteratorByPose ( double x0, double y0, double x1, double y1, int connectivity=8, bool leftToRight=false ) const {
        cv::Mat img = cvMat();
        return cv::LineIterator ( img, cvCellPoint ( x0,y0 ), cvCellPoint ( x1,y1 ), connectivity, leftToRight );
    }

    /** Returns transformation matrix
      */
    cv::Mat_<double> getTf ( ) const {
        cv::Mat_<double> m = cv::Mat_<double>::eye(3,3);
	double sx = 1./m_x_resolution;
	double sy = 1./m_x_resolution;
	m(0,0) = sx, m(0,2) = -m_x_min*sx; 
	m(1,1) = sx, m(1,2) = -m_y_min*sy; 
	return m;
    }
    /** draws a line based on metrc coordinate
      */
    void cvLine ( cv::Point2d p0, cv::Point2d p1, const cv::Scalar& color, int thickness=1, int lineType=8, int shift=0 ) {
        if ( cvtype() != -1 ) {
            cv::Mat img = cvMat ( cvtype() );
            cv::line ( img, cvCellPoint ( p0.x,p0.y ), cvCellPoint ( p1.x,p1.y ), color, thickness, lineType, shift );
        }
    }
    /// parses over all entries and looks for the min an max entry
    template <typename T1>
    void getMinMax(T1 &min, T1 &max) const{
      max = min = cellByIndex_nocheck(0);
      T* p = &m_data[0];
      T* end = &m_data[size()];
      while(p != end){
	if(*p < min) min = *p;
	if(*p > max) max = *p;
	p++;
      }
    }
    
    /** returns the current active layer
     * @returns current layer
      */
    int activeLayer() const {
      unsigned long memory_address_orgion_data = (unsigned long) m_origin_data.get();
      unsigned long memory_address_current_data = (unsigned long) m_data.get();
      unsigned long memory_address_difference = memory_address_current_data - memory_address_orgion_data;
      unsigned long memory_size_layer =  size() * getDepth();
      return memory_address_difference / memory_size_layer;
    }
    /** changes the active layer
     * @returns the new active layer
      */
    int activateLayer(size_t i) {
	m_data = (m_origin_data + size() * i);
        return activeLayer();
    }
    
    /** Erase the contents of all the cells. */
    void  clear() {
        memset ( data(), 0, this->size_total() );
    }

    
};
};


#endif //SHARED_MEM_OBJECT_GRID_MAP_H






