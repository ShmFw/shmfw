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

#ifndef SHARED_MEM_OBJECT_GRID_MAP_HEADER_H
#define SHARED_MEM_OBJECT_GRID_MAP_HEADER_H

#include <opencv/cxcore.h>

#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {


/** A 2D grid of dynamic size which stores any kind of data at each cell.
 * @tparam T The type of each cell in the 2D grid.
 * @note This class is based on the mrpt::slam::CDynamicGridMap which was published unter BSD many thanks to the mrpt team
 */
class GridMapHeader  {
private:
    double m_x_min; /// min x in metric units
    double m_x_max; /// max x in metric units
    double m_y_min; /// min y in metric units
    double m_y_max; /// max y in metric units
    double m_x_resolution;  /// resolution: metric unit = cell * resolution
    double m_y_resolution;  /// resolution: metric unit = cell * resolution
    size_t m_size_x; /// size x in cells
    size_t m_size_y; /// size y in cells
    size_t m_depth;  /// number of bytes per cell (sizeof(T))
    size_t m_layers; /// number of layers
    size_t m_size;       /// number of cells per layer
    size_t m_size_total;   /// number of cells over all layer
    size_t m_bytes;     /// number of bytes per layer
    size_t m_bytes_total;   /// number of bytes over all layer
    size_t m_type_hash_code; /// type hash code only for C+11;   
    
    /// Sets the Map size
    void initSize(const size_t size_x, const size_t size_y, const size_t depth, const size_t layers);
protected:
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t depth, const size_t layers, const size_t type_hash_code  );
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t depth, const size_t layers);
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y, const size_t depth, const size_t layers, const size_t type_hash_code);
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y, const size_t depth, const size_t layers);
    void setTypeHasCode(const size_t type_hash_code);
    template <typename T>  void setTypeHasCode(){
        size_t type_hash_code = typeid ( T ).hash_code();
	setTypeHasCode(type_hash_code);
    }
 public:

    GridMapHeader ()
        : m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_x_resolution ( 0 )
        , m_y_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )
        , m_depth ( 0 )
        , m_layers(0) 
        , m_size ( 0 )
        , m_size_total ( 0 )
        , m_bytes ( 0 )
        , m_bytes_total ( 0 )
        , m_type_hash_code ( 0 ) {
    }    
    /// Returns the horizontal size of grid map in cells count. @return m_size_x
    const size_t& getSizeX() const;
    /// Returns the horizontal size of grid map in cells count. @return m_size_x
    const size_t& getColumns() const;
    /// Returns the vertical size of grid map in cells count. @return m_size_y
    const size_t& getSizeY() const;
    /// Returns the vertical size of grid map in cells count. @return m_size_y
    const size_t& getRows() const;
    /// Returns the number of bytes per cell. @return m_depth
    const size_t& getDepth() const;
    /// Returns the number of layers. @return m_layers
    const size_t& getLayers() const;
    /// Returns the type hash code. @return m_depth 
    const size_t& getTypeHashCode() const;
    /// Returns the number of cells on one layer. @return m_size
    const size_t& size() const;
    /// Returns the number of cells on one layer. @return m_size_total
    const size_t& size_total() const;
    /// Returns the number of cells on one layer. @return m_bytes
    const size_t& bytes() const;
    /// Returns the number of cells on one layer. @return m_bytes_total
    const size_t& bytes_total() const;
    /// Returns the "x" coordinate of left side of grid map.  @return m_x_min 
    const double& getXMin() const;
    /// Returns the "x" coordinate of right side of grid map.  @return m_x_max 
    const double& getXMax() const ;
    /// Returns the "y" coordinate of top side of grid map.  @return m_y_min
    const double& getYMin() const;
    /// Returns the "y" coordinate of bottom side of grid map.  @return m_y_max 
    const double& getYMax() const;
    /// Returns the resolution of the grid map.  @return m_x_resolution 
    const double& getResolutionX() const ;
    /// Returns the resolution of the grid map.  @return m_y_resolution 
    const double& getResolutionY() const;      
    /// Transform a coordinate x values into cell index. @return cell index x
    int x2idx ( double x ) const;
    /// Transform a coordinate y values into cell index. @return cell index y
    int y2idx ( double y ) const;
    /// Transforms x y coordinates a cell index. @return index
    int cxcy2idx ( int x, int y ) const;
    /// Transform a global (linear) cell index value into its corresponding (x,y) cell indexes.
    void idx2cxcy ( const int &idx,  int &cx, int &cy ) const;
    /// Transform a cell index into a coordinate value.
    double idx2x ( int cx ) const;
    /// Transform a cell index into a coordinate value.
    double idx2y ( int cy ) const;
    /// Tests if the cell index cx, cy is in the map
    bool idxInRangeMap ( int cx, int cy ) const;
    /// Tests if the point x, y is in the map
    bool xyInRangeMap ( double x, double y ) const;

    /// tries to identify the cvtype. @return cvtype or -1
    int cvtype() const;  
    
    /// Transform a coordinate values into cell index. @param x metric value, @param y metric value  @return cell index
    cv::Point cvCellPoint (double x, double y ) const;
    /// Transform a coordinate values into cell index. @param p metric value  @return cell index
    cv::Point cvCellPoint ( const cv::Point &p ) const;
    /// Transform a coordinate values into cell index. @param p metric value  @return cell index
    cv::Point cvCellPoint ( const cv::Point2f &p ) const;
    /// Transform a coordinate values into cell index. @param p metric value  @return cell index
    cv::Point cvCellPoint ( const cv::Point2d &p ) const;
    /// Transform a cell index into a coordinate value. @param x cell index @param y cell index  @return metric value
    cv::Point2d cvPosePoint ( int &x, int &y ) const;
    /// Transform a cell index into a coordinate value. @param p cell index  @return metric value
    cv::Point2d cvPosePoint ( const cv::Point &p ) const;
    static cv::Scalar cvGreen();
    static cv::Scalar cvBlue();
    static cv::Scalar cvRed();
    
    /** Compared the entry type
     * @return true if the type T1 is equal to T
     */
    template <typename T1>
    bool isType() const {
      size_t type_hash_code =( typeid ( T1 ).hash_code() );
      return (m_type_hash_code == type_hash_code);
    }
    
  friend std::ostream& operator<< ( std::ostream &output, const GridMapHeader &o ) {
    char msg[0xFF];
    sprintf ( msg, "[%zu, %zu] @ [%4.3f, %4.3f]m/px of %zu bytes, range x:  %4.3f -> %4.3f, y: %4.3f -> %4.3f, layer %zu, type_hash_code: %zu",
	o.getSizeX(), o.getSizeY(), o.getResolutionX(), o.getResolutionY(),
	o.getDepth (),
	o.getXMin(), o.getXMax(),
	o.getYMin(), o.getYMax(),
	o.getLayers(),
	o.getTypeHashCode() );
    output << msg;
    return output;
  }
  friend std::istream& operator>> ( std::istream &input, GridMapHeader &o ) {
    return input;
  }
};
};


#endif //SHARED_MEM_OBJECT_GRID_MAP_HEADER_H





