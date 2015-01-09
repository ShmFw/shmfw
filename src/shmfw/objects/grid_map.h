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

#include <stdio.h>
#include <opencv/cxcore.h>
#include <shmfw/objects/grid_map_header.h>
#include <boost/interprocess/offset_ptr.hpp>

namespace ShmFw {

/** A 2D grid with data pointers able to handle shared memory objects
 * @note This class is based on the mrpt::slam::CDynamicGridMap which was published unter BSD many thanks to the mrpt team
 * @tparam T grid cell type
 * @tparam TPtr pointer type a boost::interprocess::offset_ptr<T> is neede for to use the grid in shm otherwise T* is sufficient 
 **/
template <typename T, class TPtr = T* >
class GridMap : public ShmFw::GridMapHeader {
protected:
    TPtr m_origin_data; /// cells
    TPtr m_data;        /// cells
public:
    /// constructor
    GridMap ()
        : GridMapHeader ( )
        , m_data () {
    }
    /// constructor
    GridMap ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t layers, T *data, const T * fill_value = NULL ) {
        size_t type_hash_code;
#if __cplusplus > 199711L
        type_hash_code = typeid ( T ).hash_code();
#else
        type_hash_code = 0;
#endif
        initHeader ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, sizeof ( T ), layers, type_hash_code );
        initData ( data, fill_value );
    }
    /// Initilialies a given data array
    void initData ( T *data, const T * fill_value = NULL ) {
        m_data = data;
        if ( fill_value ) fill ( fill_value );
    }
    /** copies data to layer from src
     * @param src source
     * @param layer layer
     **/
    void copyDataToLayerFromArray ( const void *src, int layer ) {
        memcpy ( data_layer(), src, bytes() );
    }
    /** copies data between layers from src
     * @param src source
     * @param layer layer
     **/
    template<class TPtrDes>
    void copyDataToLayer ( GridMap<T, TPtrDes> & des, int destination_layer, int source_layer ) {
        memcpy ( des.data_layer(destination_layer), data_layer(source_layer), bytes() );
    }
    /** copies all layers layer to des
     * @param des destination
     **/
    template<typename T1> void copyDataTo ( T1& des ) const {
        memcpy ( &des[0], origin_data(), bytes_total() );
    }
    /** copies all layers from src.
     * @param src source
     **/
    template<typename T1> GridMap& copyDataFrom ( const T1& src ) {
        memcpy ( origin_data(), &src[0], bytes_total() );
        return *this;
    }
    /** copies all layers from src. 
     * @param src source
     **/
    void copyDataFromArray ( const void *src ) {
        memcpy ( origin_data(), src, bytes_total() );
    }

    /** Fills all the cells with the same value using memset
     * @param value element to fill the current layer
     **/
    inline void set ( unsigned char value ) {
        size_t size = this->size() * this->getDepth();
        memset(data(), value, size);
    }
    
    
    /** Fills all the cells with the same value
     * @note if one liles ot set the value to zero one shouel use memset
     * @param value element to fill the current layer
     **/
    inline void fill ( const T& value ) {
        T *p = data();
        T *end = p+size();
        while ( p != end ) {
            *p++ = value;
        }
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return cell elment of the active layer
     **/
    const T &operator[] ( int idx ) const {
        return m_data[idx]; 
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return cell elment of the active layer
     **/
    T &operator[] ( int idx ) {
        return m_data[idx];
    }
    /** Returns a reference to a after the last element of the current active layer.
     * @return pointer to the element after the current active layer
     **/
    const T *end() const {
        return &m_data[size()]; 
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the current active layer (data) 
     **/
    const T *data() const {
        return &m_data[0]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the current active layer (data) 
     **/
    T *data() {
        return &m_data[0]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the current active layer (data) with offset i
     **/
    T *data ( size_t i ) {
        return &m_data[i]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the current active layer (data) with offset i
     **/
    const T *data ( size_t i ) const {
        return &m_data[i]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the orgin of all layers (data)
     **/
    T *origin_data() {
        return &m_origin_data[0]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the orgin of all layers (data)
     **/
    const T *origin_data() const {
        return &m_origin_data[0]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the orgin of all layers (data) with offset i
     **/
    T *origin_data ( size_t i ) {
        return &m_origin_data[i]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a reference to a cell, no boundary checks are performed.
     * @return pointer to the orgin of all layers (data) with offset i
     **/
    const T *origin_data ( size_t i ) const {
        return &m_origin_data[i]; ///@note the acces with the brackets is needed to be compatible with the boost offset_ptr
    }
    /** Returns a pointer to a layer 
     * @return pointer ot layer data
     **/
    T* data_layer ( size_t layer ) {
        if ( layer >= getLayers() ) throw 0;
        return origin_data ( size() * layer );
    }
    /** Returns a pointer to a layer 
     * @return pointer ot layer data
     **/
    const T* data_layer ( size_t layer ) const {
        if ( layer >= getLayers() ) throw 0;
        return origin_data ( this->size() * layer );
    }
    /** Returns a pointer to a layer ending
     * @param layer
     * @return pointer to cell after after the layer
     **/
    T* end_layer ( size_t layer ) {
        if ( layer >= getLayers() ) throw 0;
        return origin_data ( size() * ( layer+1 ) );
    }
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline T& cellByIndex_nocheck ( int idx ) {
        return m_data[idx];
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline const T& cellByIndex_nocheck ( int idx ) const {
        return m_data[idx];
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline T& cellByIndex_nocheck ( int cx, int cy ) {
        return cellByIndex_nocheck( cxcy2idx ( cx,cy ) );
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline const T& cellByIndex_nocheck ( int cx, int cy ) const {
        return cellByIndex_nocheck( cxcy2idx ( cx,cy ) );
    };

    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline T& cellByIndex_nocheck ( const cv::Point &p ) {
        return cellByIndex_nocheck ( p.x, p.y );
    };

    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline const T& cellByIndex_nocheck ( const cv::Point &p ) const {
        return cellByIndex_nocheck ( p.x, p.y );
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline T& layerCellByIndex_nocheck ( int idx, int layer ) {
        return m_origin_data[idx + this->size() * layer];
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline const T& layerCellByIndex_nocheck ( int idx, int layer ) const {
        return m_origin_data[idx + this->size() * layer];
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline T& layerCellByIndex_nocheck ( int cx, int cy, int layer  ) {
        return layerCellByIndex_nocheck( this->cxcy2idx ( cx,cy ), layer );
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline const T& layerCellByIndex_nocheck ( int cx, int cy, int layer ) const {
        return layerCellByIndex_nocheck( this->cxcy2idx ( cx,cy ), layer );
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline T& layerCellByIndex_nocheck ( const cv::Point &p, int layer  ) {
        return layerCellByIndex_nocheck ( p.x, p.y, layer  );
    };
    /// Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
    inline const T& layerCellByIndex_nocheck ( const cv::Point &p, int layer  ) const {
        return layerCellByIndex_nocheck ( p.x, p.y, layer );
    };

    /// Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
    inline T& cellByPos ( double x, double y ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) throw 0;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) throw 0;

        return cellByIndex_nocheck ( cx, cy );
    }
    /// Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
    inline T& layerCellByPos ( double x, double y, int layer ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) throw 0;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) throw 0;
        if ( layer<0 || layer>=static_cast<int> ( getLayers() ) ) throw 0;

        return layerCellByIndex_nocheck ( cx, cy, layer );
    }

    /// Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
    inline const T& cellByPos ( double x, double y ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) throw 0;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) throw 0;

        return cellByIndex_nocheck ( cx, cy );
    }
    /// Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
    inline const T& layerCellByPos ( double x, double y, int layer ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) throw 0;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) throw 0;
        if ( layer<0 || layer>=static_cast<int> ( getLayers() ) ) throw 0;

        return layerCellByIndex_nocheck ( cx, cy, layer );
    }
    /// set the contents of a cell given by its coordinates if the coordinates are with the range.
    inline void setCellByPos ( double x, double y, const T &src ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return;

        cellByIndex_nocheck ( cx, cy ) = src;
    }
    /// set the contents of a cell given by its coordinates if the coordinates are with the range.
    inline void setLayerCellByPos ( double x, double y, int layer, const T &src ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return;
        if ( layer<0 || layer>=static_cast<int> ( getLayers() ) ) return;

        layerCellByIndex_nocheck ( cx, cy, layer ) = src;
    }
    /// Gets the contents of a cell given by its coordinates if the coordinates are with the range.
    inline void getCellByPos ( double x, double y, T &des ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return;

        des = cellByIndex_nocheck ( cx, cy );
    }
    /// Gets the contents of a cell given by its coordinates if the coordinates are with the range.
    inline void getLayerCellByPos ( double x, double y, int layer, T &des ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( getSizeX() ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( getSizeY() ) ) return;
        if ( layer<0 || layer>=static_cast<int> ( getLayers() ) ) return;

        des = layerCellByIndex_nocheck ( cx, cy, layer );
    }

    /// Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
    inline  T* cellByIndex ( unsigned int cx, unsigned int cy ) {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return NULL;
        else    return &cellByIndex_nocheck ( cx, cy );
    }
    /// Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
    inline  T* layerCellByIndex ( unsigned int cx, unsigned int cy, unsigned int layer ) {
        if ( cx>=getSizeX() || cy>=getSizeY() || layer>=getLayers() )
            return NULL;
        else    return &layerCellByIndex_nocheck ( cx, cy, layer );
    }

    /// Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
    inline const T* cellByIndex ( unsigned int cx, unsigned int cy ) const {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return NULL;
        else    return &cellByIndex_nocheck ( cx, cy );
    }
    /// Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
    inline const T* layerCellByIndex ( unsigned int cx, unsigned int cy, unsigned int layer  ) const {
        if ( cx>=getSizeX() || cy>=getSizeY() || layer>=getLayers() )
            return NULL;
        else    return &layerCellByIndex_nocheck ( cx, cy, layer );
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void setCellByIndex ( unsigned int cx, unsigned int cy, const T &src ) {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return;
        else
            cellByIndex_nocheck ( cx, cy ) = src;
    }
    /// set the contents of a cell given by its cell indexes if the indexes are with the range.
    inline void setLayerCellByIndex ( unsigned int cx, unsigned int cy, unsigned int layer , const T &src ) {
        if ( cx>=getSizeX() || cy>=getSizeY() || layer>=getLayers() )
            return;
        else
            layerCellByIndex_nocheck ( cx, cy, layer ) = src;
    }
    /// set the contents of a cell given by its cell indexes if the indexes are with the range.
    inline void getCellByIndex ( unsigned int cx, unsigned int cy, T &des ) const {
        if ( cx>=getSizeX() || cy>=getSizeY() )
            return;
        else
            des = cellByIndex_nocheck ( cx, cy );
    }
    /// set the contents of a cell given by its cell indexes if the indexes are with the range.
    inline void getLayerCellByIndex ( unsigned int cx, unsigned int cy, unsigned int layer, T &des ) const {
        if ( cx>=getSizeX() || cy>=getSizeY() || layer>=getLayers() )
            return;
        else
            des = layerCellByIndex_nocheck ( cx, cy, layer );
    }
    ///Returns a reference to a cell, no boundary checks are performed.
    const T &operator() ( int cx, int cy ) const {
        return cellByIndex_nocheck ( cx, cy );
    }
    /// Returns a reference to a cell, no boundary checks are performed.
    T &operator() ( int cx, int cy ) {
        return cellByIndex_nocheck ( cx, cy );
    }
    /// Returns the data as opencv matrix
    cv::Mat_<T> cvMatLayer ( size_t layer ) {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), data_layer ( layer ) );
    }
    /// Returns the data as opencv matrix
    const cv::Mat_<T> cvMatLayer ( size_t layer ) const {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), ( T* ) data_layer ( layer ) );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMat() {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), data() );
    }
    /// Returns the data as opencv matrix
    cv::Mat cvMat ( int cvtype ) {
        return cv::Mat ( getSizeY(), getSizeX(), cvtype, data() );
    }
    /// Returns the data as opencv matrix
    cv::Mat_<T> &cvMatTotal ( cv::Mat_<T> &m ) {
        m = cvMatTotal();
        return m;
    }
    /// Returns the data as opencv matrix
    cv::Mat_<T> cvMatTotal() const {
        return cv::Mat_<T> ( getSizeY() *getLayers(), getSizeX(), ( T* ) origin_data() );
    }
    /// creates a opencv line iterator based on a metric start and endpoint
    cv::LineIterator cvLineIteratorByPose ( double x0, double y0, double x1, double y1, int connectivity=8, bool leftToRight=false ) const {
        cv::Mat img = cvMat();
        return cv::LineIterator ( img, cvCellPoint ( x0,y0 ), cvCellPoint ( x1,y1 ), connectivity, leftToRight );
    }

    /// draws a line based on metrc coordinate
    void cvLine ( cv::Point2d p0, cv::Point2d p1, const cv::Scalar& color, int thickness=1, int lineType=8, int shift=0 ) {
        if ( cvtype() != -1 ) {
            cv::Mat img = cvMat ( cvtype() );
            cv::line ( img, cvCellPoint ( p0.x,p0.y ), cvCellPoint ( p1.x,p1.y ), color, thickness, lineType, shift );
        }
    }
    /// parses over all entries and looks for the min an max entry
    template <typename T1>
    void getMinMax ( T1 &min, T1 &max ) const {
        max = min = cellByIndex_nocheck ( 0 );
        T* p = &m_data[0];
        T* end = &m_data[size()];
        while ( p != end ) {
            if ( *p < min ) min = *p;
            if ( *p > max ) max = *p;
            p++;
        }
    }

    /** returns the current active layer
     * @returns current layer
     **/
    int activeLayer() const {
        unsigned long memory_address_orgion_data = ( unsigned long ) origin_data();
        unsigned long memory_address_current_data = ( unsigned long ) data();
        unsigned long memory_address_difference = memory_address_current_data - memory_address_orgion_data;
        unsigned long memory_size_layer =  size() * getDepth();
        return memory_address_difference / memory_size_layer;
    }
    /** changes the active layer
     * @returns the new active layer
     **/
    int activateLayer ( size_t i ) {
        m_data = ( m_origin_data + size() * i );
        return activeLayer();
    }

    /// Erase the contents of all the cells. 
    void  clear() {
        memset ( data(), 0, this->size_total() );
    }

    /**
     * creates a matlab struct
     * @param output output stream
     * @param varaible name of the matlab struct
     * @code
     * % c++ 
     * std::ofstream outfile("/tmp/gridmap.m", std::ofstream::out);
     * grid.matlab(outfile, "grid");
     * outfile.close();   
     * % matlab
     * run('/tmp/gridmap.m')
     * surf(grid.x,grid.y,reshape(grid.data(1,:,:), grid.size_x, grid.size_y))
     * @endcode
     **/
    std::ostream& matlab ( std::ostream &output, const std::string &variable ) const {
        GridMapHeader::matlab ( output, variable );
        for ( size_t l = 0; l < this->getLayers(); l++ ) {
            const T *p = this->data_layer(l);
            output << variable << ".data(" << l+1 << ",:,:) = [";
            for ( size_t y = 0; y < this->getSizeY(); y++ ) {
                output << std::endl;
                for ( size_t x = 0; x < this->getSizeX(); x++ ) {
                    output << std::setw ( 20 ) << *p++;
                    if ( x < this->getSizeX()-1 ) output << ",";
                }
                if ( y < this->getSizeY()-1 ) output << ";";
            }
            output << "]\n";
        }
        return output;
    }
};

/** placing a shared grid into the shared memory requiers a boost::interprocess::offset_ptr<T>
 *  this using is a short cut for ShmFw::Var<ShmFw::GridMap<T, boost::interprocess::offset_ptr<T> > 
 **/
template <typename T> using GridMapInterprocess = GridMap<T,  boost::interprocess::offset_ptr<T> >;

};

#endif ///SHARED_MEM_OBJECT_GRID_MAP_H




