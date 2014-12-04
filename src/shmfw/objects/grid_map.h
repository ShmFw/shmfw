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
#include <boost/interprocess/offset_ptr.hpp>

#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {


/** A 2D grid of dynamic size which stores any kind of data at each cell.
 * @tparam T The type of each cell in the 2D grid.
 * @note This class is based on the mrpt::slam::CDynamicGridMap which was published unter BSD many thanks to the mrpt team
 */
template <typename T>
class GridMap  {
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
    boost::interprocess::offset_ptr<T>  m_data; /// cells

    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void setBounderies ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution ) {
        m_x_min = x_resolution*round ( x_min/x_resolution );
        m_y_min = y_resolution*round ( y_min/y_resolution );
        m_x_max = x_resolution*round ( x_max/x_resolution );
        m_y_max = y_resolution*round ( y_max/y_resolution );
        // Res:
        m_x_resolution = x_resolution;
        m_y_resolution = y_resolution;
        // Now the number of cells should be integers:
        m_size_x = round ( ( m_x_max-m_x_min ) /m_x_resolution );
        m_size_y = round ( ( m_y_max-m_y_min ) /m_y_resolution );
    }
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void setBounderies ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y ) {
        m_x_min = x_min;
        m_y_min = y_min;
        m_x_max = x_max;
        m_y_max = y_max;
	m_size_x = size_x;
	m_size_y = size_y;
        m_x_resolution = (m_x_max-m_x_min) / (double) m_size_x;
	m_y_resolution = (m_y_max-m_y_min) / (double) m_size_y;
	
	//setBounderies ( x_min, x_max, y_min, y_max, m_x_resolution, m_y_resolution );
    }
public:
    GridMap ()
        : m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_x_resolution ( 0 )
        , m_y_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )
        , m_depth ( sizeof ( T ) )
#if __cplusplus > 199711L
        , m_type_hash_code ( typeid ( T ).hash_code() )
#else
        ,m_type_hash_code ( 0 )
#endif
        , m_data () {
    }
    GridMap ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, T *data, const T * fill_value = NULL ) {
        init ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, data, fill_value );
    }
    /** Initilialies a given data array
      */
    void init (
        const double x_min, const double x_max,
        const double y_min, const double y_max,
        const double x_resolution,
        const double y_resolution, T *data, const T * fill_value = NULL ) {
        setBounderies ( x_min, x_max, y_min, y_max, x_resolution, y_resolution, data, fill_value );
        m_data = data;
        m_depth = sizeof ( T );
#if __cplusplus > 199711L
        m_type_hash_code = typeid ( T ).hash_code();
#else
        m_type_hash_code = 0;
#endif
        if ( fill_value ) fill ( fill_value );
    }
    template<typename T1>
    void copyDataTo ( T1& des ) const {
        /// works as long as the data is aligned
        if ( size() != des.size() ) throw 0;
        memcpy ( &des[0], data(), sizeof ( T ) * size() );
    }
    template<typename T1>
    GridMap& copyDataFrom ( const T1& src ) {
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
    /** Returns the number of cells.
      */
    size_t size() const {
        return m_size_x*m_size_y;
    }

    /** Returns the horizontal size of grid map in cells count.
        */
    inline size_t getSizeX() const {
        return m_size_x;
    }
    inline size_t getColumns() const {
        return getSizeX();
    }

    /** Returns the vertical size of grid map in cells count.
        */
    inline size_t getSizeY() const {
        return m_size_y;
    }
    inline size_t getRows() const {
        return getSizeY();
    }
    /** Returns the number of bytes per cell
        */
    inline size_t getDepth() const {
        return m_depth;
    }
    /** Returns the type hash code
     */
    inline size_t getTypeHashCode() const {
        return m_type_hash_code;
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
    inline double getResolutionX() const  {
        return m_x_resolution;
    }
    /** Returns the resolution of the grid map.
        */
    inline double getResolutionY() const  {
        return m_y_resolution;
    }

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& cellByIndex_nocheck ( int idx )  {
        return m_data[idx];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline const T& cellByIndex_nocheck ( int idx ) const {
        return m_data[idx];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
    inline T& cellByIndex_nocheck ( int cx, int cy ) {
        return m_data[ cx + cy*this->m_size_x ];
    };

    /** Returns a pointer to the contents of a cell given by its cell indexes, no checks are performed.
      */
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
    /** Transform a coordinate values into cell indexes.
      */
    inline int   x2idx ( double x ) const {
        return static_cast<int> ( ( x-this->m_x_min ) /this->m_x_resolution );
    }
    inline int   y2idx ( double y ) const {
        return static_cast<int> ( ( y-this->m_y_min ) /this->m_y_resolution );
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
        return this->m_x_min+ ( cx+0.5f ) *this->m_x_resolution;
    }
    inline double   idx2y ( int cy ) const {
        return this->m_y_min+ ( cy+0.5f ) *this->m_y_resolution;
    }

    /** Transform a coordinate value into a cell index, using a diferent "x_min" value
        */
    inline int   x2idx ( double x,double x_min ) const {
        SHMFW_UNUSED_PARAM ( x_min );
        return static_cast<int> ( ( x-this->m_x_min ) / this->m_x_resolution );
    }
    inline int   y2idx ( double y, double y_min ) const {
        SHMFW_UNUSED_PARAM ( y_min );
        return static_cast<int> ( ( y-this->m_y_min ) /this->m_y_resolution );
    }

    /** The user must implement this in order to provide "saveToTextFile" a way to convert each cell into a numeric value */
    double cell2double ( const T& c ) const {
        SHMFW_UNUSED_PARAM ( c );
        return 0;
    }
    friend std::ostream& operator<< ( std::ostream &output, const GridMap &o ) {
        char msg[0xFF];
        sprintf ( msg, "[%zu, %zu] @ [%4.3f, %4.3f]m/px of %zu bytes, range x:  %4.3f -> %4.3f, y: %4.3f -> %4.3f, type_hash_code: %zu",
                  o.getSizeX(), o.getSizeY(), o.getResolutionX(), o.getResolutionY(),
                  o.getDepth (),
                  o.getXMin(), o.getXMax(),
                  o.getYMin(), o.getYMax(),
                  o.getTypeHashCode() );
        output << msg;
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, GridMap &o ) {
        return input;
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
    /** Returns a metric pose as cell pose.
      */
    inline cv::Point cvCellPoint (double x, double y ) const {
        return cv::Point ( x2idx ( x ), y2idx ( y ) );
    }
    /** Returns a metric pose as cell pose.
      */
    template <typename T1>
    inline cv::Point cvCellPoint ( cv::Point_<T1> p ) {
        return cv::Point ( x2idx ( p.x ), y2idx ( p.y ) );
    }
    /** Returns a cell pose as metric pose.
      */
    inline cv::Point2d cvPosePoint ( int &x, int &y ) {
        return cv::Point2d ( idx2x ( x ), idx2y ( y ) );
    }
    /** Returns a cell pose as metric pose.
      */
    inline cv::Point2d cvPosePoint ( const cv::Point &p ) {
        return cv::Point2d ( idx2x ( p.x ), idx2y ( p.y ) );
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
        return cv::Mat_<T> ( getSizeY(), getSizeX(), m_data.get() );
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
        return cv::Mat ( getSizeY(), getSizeX(), cvtype, m_data.get() );
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
    /** tries to identify the cvtype
     * @return cvtype or -1
      */
    int cvtype() const {
        int type = -1;
        if ( m_depth == 1 ) type = CV_8U;
        else if ( m_depth == 2 ) type = CV_16U;
        else if ( m_depth == 3 ) type = CV_8UC3;
        return type;
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
};
};


#endif //SHARED_MEM_OBJECT_GRID_MAP_H





