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

#ifndef SHARED_MEM_OBJECT_GRID_LAYER_MAP_H
#define SHARED_MEM_OBJECT_GRID_LAYER_MAP_H

#include <opencv/cxcore.h>
#include <shmfw/objects/grid_map.h>

#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {


/** A 2D grid of dynamic size which stores any kind of data at each cell.
 * @tparam T The type of each cell in the 2D grid.
 * @note This class is based on the mrpt::slam::CDynamicGridMap which was published unter BSD many thanks to the mrpt team
 */
class GridLayerMapHeader  {
protected:
    double m_x_min; /// min x in metric units
    double m_x_max; /// max x in metric units
    double m_y_min; /// min y in metric units
    double m_y_max; /// max y in metric units
    double m_x_resolution;  /// resolution: metric unit = cell * resolution
    double m_y_resolution;  /// resolution: metric unit = cell * resolution
    size_t m_size_x; /// size x in cells
    size_t m_size_y; /// size y in cells
    size_t m_depth;  /// number of bytes per cell (sizeof(T))
    size_t m_type_hash_code; /// type hash code only for C+11;
    size_t m_layers; /// number of layers
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void setBounderies ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution );
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void setBounderies ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y );
public:

    GridLayerMapHeader ()
        : m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_x_resolution ( 0 )
        , m_y_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )
        , m_depth ( 0 )
        , m_type_hash_code ( 0 )
        , m_layers(0)  {
    }    
    /// Returns the horizontal size of grid map in cells count. @return m_size_x
    size_t getSizeX() const;
    /// Returns the horizontal size of grid map in cells count. @return m_size_x
    size_t getColumns() const;
    /// Returns the vertical size of grid map in cells count. @return m_size_y
    size_t getSizeY() const;
    /// Returns the vertical size of grid map in cells count. @return m_size_y
    size_t getRows() const;
    /// Returns the number of bytes per cell. @return m_depth
    size_t getDepth() const;
    /// Returns the number of layers. @return m_layers
    size_t getLayers() const;
    /// Returns the type hash code. @return m_depth 
    size_t getTypeHashCode() const;
    /// Returns the "x" coordinate of left side of grid map.  @return m_x_min 
    double getXMin() const;
    /// Returns the "x" coordinate of right side of grid map.  @return m_x_max 
    double getXMax() const ;
    /// Returns the "y" coordinate of top side of grid map.  @return m_y_min
    double getYMin() const;
    /// Returns the "y" coordinate of bottom side of grid map.  @return m_y_max 
    double getYMax() const;
    /// Returns the resolution of the grid map.  @return m_x_resolution 
    double getResolutionX() const ;
    /// Returns the resolution of the grid map.  @return m_y_resolution 
    double getResolutionY() const;      
    /// Transform a coordinate x values into cell index. @return cell index x
    int x2idx ( double x ) const;
    /// Transform a coordinate y values into cell index. @return cell index y
    int y2idx ( double y ) const;
    /// Transforms x y coordinates a cell index. @return index
    int xy2idx ( double x, double y ) const;
    /// Transform a global (linear) cell index value into its corresponding (x,y) cell indexes.
    void idx2cxcy ( const int &idx,  int &cx, int &cy ) const;
    /// Transform a cell index into a coordinate value.
    double idx2x ( int cx ) const;
    /// Transform a cell index into a coordinate value.
    double idx2y ( int cy ) const;
    /// Transform a coordinate value into a cell index, using a diferent "x_min" value
    int x2idx ( double x,double x_min ) const;
    /// Transform a coordinate value into a cell index, using a diferent "y_min" value
    int y2idx ( double y, double y_min ) const;

    /// tries to identify the cvtype. @return cvtype or -1
    int cvtype() const;  
    
    cv::Point cvCellPoint (double x, double y ) const;
    cv::Point cvCellPoint ( const cv::Point &p ) const;
    cv::Point cvCellPoint ( const cv::Point2f &p ) const;
    cv::Point cvCellPoint ( const cv::Point2d &p ) const;
    cv::Point2d cvPosePoint ( int &x, int &y ) const;
    cv::Point2d cvPosePoint ( const cv::Point &p ) const;
    
    /// Returns the number of cells on one layer. @return m_size_x*m_size_y
    size_t size() const;
    friend std::ostream& operator<< ( std::ostream &output, const GridLayerMapHeader &o );
    friend std::istream& operator>> ( std::istream &input, GridLayerMapHeader &o );
};
template <typename T>
class GridLayerMap : public GridLayerMapHeader {
protected:
    T *m_origin_data; /// pointer to fist layer
    T *m_data; /// cells  

public:
    GridLayerMap () 
        : GridLayerMapHeader()
        , m_origin_data(NULL)
        , m_data (NULL) {
#if __cplusplus > 199711L
        m_type_hash_code  =  typeid ( T ).hash_code();
#else
        m_type_hash_code =  0;
#endif
    }
    GridLayerMap ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t layers, T *data = NULL) {
        initWithResolution ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, layers, data);
    }
    /** Initilialies a given data array
      */
    void initWithResolution (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double x_resolution, const double y_resolution, 
	const size_t layers,
	T *data = NULL) {
        setBounderies ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, data);
        m_depth = sizeof ( T );
#if __cplusplus > 199711L
        m_type_hash_code = typeid ( T ).hash_code();
#else
        m_type_hash_code = 0;
#endif
	m_layers = layers;
	m_origin_data = data;
	activateLayer(0);
    }
    /** Changes the size of the grid
      */
    void  initWithNrOfCells (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const size_t size_x, const size_t size_y,
	const size_t layers,
	T *data = NULL) {
        this->setBounderies ( x_min, x_max, y_min, y_max, size_x,  size_y);

        m_depth = sizeof ( T );
#if __cplusplus > 199711L
        m_type_hash_code = typeid ( T ).hash_code();
#else
        m_type_hash_code = 0;
#endif
	m_layers = layers;
	m_origin_data = data;
	activateLayer(0);
    }
    template<typename T1>
    void copyDataTo ( T1& des ) const {
        /// works as long as the data is aligned
        if ( size() != des.size() ) throw 0;
        memcpy ( &des[0], data(), sizeof ( T ) * size() );
    }
    template<typename T1>
    GridLayerMap& copyDataFrom ( const T1& src ) {
        /// works as long as the data is aligned
        if ( size() != src.size() ) throw 0;
        memcpy ( data(), &src[0], sizeof ( T ) * size() );
        return *this;
    }
    void copyDataFromArray ( const void *src ) {
        /// works as long as the data is aligned
        memcpy ( data(), src, sizeof ( T ) * size() );
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


    /** Returns a pointer to the contents of a cell given by its cell indexes, 
     * no checks are performed.
     * @param idx index
     * @return reference to cell context 
     **/
    inline T& cellByIndex_nocheck ( int idx )  {
        return m_data[idx];
    };
    /** Returns a pointer to the contents of a cell given by its cell indexes, 
     * no checks are performed.
     * @param idx index
     * @return reference to cell context 
     **/
    inline const T& cellByIndex_nocheck ( int idx ) const {
        return m_data[idx];
    };
    /** Returns a pointer to the contents of a cell given by its cell indexes, 
     *  no checks are performed.
     * @param cx 
     * @param cy 
     * @return reference to cell context 
     **/
    inline T& cellByIndex_nocheck ( int cx, int cy ) {
        return m_data[ cx + cy*this->m_size_x ];
    };
    /** Returns a pointer to the contents of a cell given by its cell indexes, 
     *  no checks are performed.
     * @param cx 
     * @param cy 
     * @return reference to cell context 
     **/
    inline const T& cellByIndex_nocheck ( int cx, int cy ) const {
        return m_data[ cx + cy*this->m_size_x ];
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

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return NULL;

        return cellByIndex_nocheck ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its coordinates, or NULL if it is out of the map extensions.
      */
    inline const T& cellByPos ( double x, double y ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return NULL;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return NULL;

        return cellByIndex_nocheck ( cx, cy );
    }
    /** set the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void setCellByPos ( double x, double y, const T &src ) {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return;

        cellByIndex_nocheck ( cx, cy ) = src;
    }
    /** Gets the contents of a cell given by its coordinates if the coordinates are with the range.
      */
    inline void getCellByPos ( double x, double y, T &des ) const {
        int cx = x2idx ( x );
        int cy = y2idx ( y );

        if ( cx<0 || cx>=static_cast<int> ( this->m_size_x ) ) return;
        if ( cy<0 || cy>=static_cast<int> ( this->m_size_y ) ) return;

        des = cellByIndex_nocheck ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline  T*	cellByIndex ( unsigned int cx, unsigned int cy ) {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return NULL;
        else	return &cellByIndex_nocheck ( cx, cy );
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, or NULL if it is out of the map extensions.
      */
    inline const T* cellByIndex ( unsigned int cx, unsigned int cy ) const {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return NULL;
        else	return &cellByIndex_nocheck ( cx, cy );
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void setCellByIndex ( unsigned int cx, unsigned int cy, const T &src ) {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
            return;
        else
            cellByIndex_nocheck ( cx, cy ) = src;
    }
    /** set the contents of a cell given by its cell indexes if the indexes are with the range.
      */
    inline void getCellByIndex ( unsigned int cx, unsigned int cy, T &des ) const {
        if ( cx>=this->m_size_x || cy>=this->m_size_y )
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
    cv::Mat_<T> &cvMatAllLayers ( cv::Mat_<T> &m ) {
        m = cvMatAllLayers();
        return m;
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMatAllLayers() const {
        return cv::Mat_<T> ( getSizeY()*getLayers(), getSizeX(), (T*) m_origin_data );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> &cvMat ( cv::Mat_<T> &m ) {
        m = cvMat();
        return m;
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat_<T> cvMat() const {
        return cv::Mat_<T> ( getSizeY(), getSizeX(), m_data );
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat &cvMat ( cv::Mat &m, int cvtype ) {
        m = cvMat ( cvtype );
        return m;
    }
    /** Returns the data as opencv matrix
      */
    cv::Mat cvMat ( int cvtype ) const {
        return cv::Mat ( getSizeY(), getSizeX(), cvtype, m_data );
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
    
    /** returns the current active layer
     * @returns current layer
      */
    int activeLayer() const {
      unsigned long memory_address_orgion_data = (unsigned long) m_origin_data;
      unsigned long memory_address_current_data = (unsigned long) m_data;
      unsigned long memory_address_difference = memory_address_current_data - memory_address_orgion_data;
      unsigned long memory_size_layer =  size() * m_depth;
      return memory_address_difference / memory_size_layer;
    }
    /** changes the active layer
     * @returns the new active layer
      */
    int activateLayer(size_t i) {
	m_data = (m_origin_data + size() * i);
        return activeLayer();
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
    /// return a opencv color for drawing
    static cv::Scalar cvGreen() {
        return cv::Scalar ( 0,255,0 );
    }
    /// return a opencv color for drawing
    static cv::Scalar cvBlue() {
        return cv::Scalar ( 0,0,255 );
    }
    /// return a opencv color for drawing
    static cv::Scalar cvRed() {
        return cv::Scalar ( 0,0,255 );
    }
    /** Compared the entry type
     * @return true if the type T1 is equal to T
     */
    template <typename T1>
    bool isType() const {
      size_t type_hash_code =( typeid ( T1 ).hash_code() );
      return (m_type_hash_code == type_hash_code);
    }
    /// allocates data
    void allocate() {
	m_origin_data = new T[size()*m_layers*m_depth];
	activateLayer(0);
    }
    /// releases allocated data
    void release() {
      if(m_origin_data != NULL){
	delete m_origin_data;
      }
    }
};
};


#endif //SHARED_MEM_OBJECT_GRID_LAYER_MAP_H





