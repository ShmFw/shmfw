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

#include <iomanip>
#include <opencv/cxcore.h>
#include <typeinfo>  // Needed for typeid()

#define SHMFW_UNUSED_PARAM(a)		(void)(a)
namespace ShmFw {


/** A general 2D grid header without data pointer
 * @note This class is based on the mrpt::slam::CDynamicGridMap which was published unter BSD many thanks to the mrpt team
 */
class GridMapHeader  {
private:
    double m_x_min; /// min x in metric units
    double m_x_max; /// max x in metric units
    double m_y_min; /// min y in metric units
    double m_y_max; /// max y in metric units
    double m_rotation; /// rotation of the map
    double m_x_resolution;  /// resolution: metric unit = cell * resolution
    double m_y_resolution;  /// resolution: metric unit = cell * resolution
    double m_M[2][3];    /// Transform p[pixel memory] = Mint * Mext * p[m world] 
    double m_Minv[2][3]; /// Transform p[m world] = Mint * Mext * p[pixel memory]
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
    void initSize ( const size_t size_x, const size_t size_y, const size_t depth, const size_t layers );
protected:
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t depth, const size_t layers, const size_t type_hash_code );
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution, const size_t depth, const size_t layers );
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y, const size_t depth, const size_t layers, const size_t type_hash_code );
    /// Sets the bounderies and rounds the values to integers and to multipliers of the given resolution
    void initHeader ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y, const size_t depth, const size_t layers );
    void setTypeHasCode ( const size_t type_hash_code );
    template <typename T>  void setTypeHasCode() {
        size_t type_hash_code = typeid ( T ).hash_code();
        setTypeHasCode ( type_hash_code );
    }
public:

    GridMapHeader ()
        : m_x_min ( 0 )
        , m_x_max ( 0 )
        , m_y_min ( 0 )
        , m_y_max ( 0 )
        , m_rotation ( 0 )
        , m_x_resolution ( 0 )
        , m_y_resolution ( 0 )
        , m_size_x ( 0 )
        , m_size_y ( 0 )
        , m_depth ( 0 )
        , m_layers ( 0 )
        , m_size ( 0 )
        , m_size_total ( 0 )
        , m_bytes ( 0 )
        , m_bytes_total ( 0 )
        , m_type_hash_code ( 0 ) {
    }
    /// Changes the resolution and thus boundaries of tthe maps while keeping their size constant
    void updateResolution (const double x_resolution, const double y_resolution);
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
    /// Returns the number of cells over all layers. @return m_size_total
    const size_t& size_total() const;
    /// Returns the number of bytes on one layer. @return m_bytes
    const size_t& bytes() const;
    /// Returns the number of bytes over all layers. @return m_bytes_total
    const size_t& bytes_total() const;
    /// Returns the "x" coordinate of left side of grid map.  @return m_x_min
    const double& getXMin() const;
    /// Returns the "x" coordinate of right side of grid map.  @return m_x_max
    const double& getXMax() const ;
    /// Returns the "y" coordinate of top side of grid map.  @return m_y_min
    const double& getYMin() const;
    /// Returns the "y" coordinate of bottom side of grid map.  @return m_y_max
    const double& getYMax() const;
    /// Returns the rotation of grid map.  @return m_rotation
    const double& getRotation() const;
    /// sets the rotation @param phi
    void setRotation(double phi);
    /// Returns the resolution of the grid map.  @return m_x_resolution
    const double& getResolutionX() const ;
    /// Returns the resolution of the grid map.  @return m_y_resolution
    const double& getResolutionY() const;
    /// Returns the resolution of the grid map. it thows an exception if m_x_resolution != m_y_resolution  @return resolution
    const double& getResolution() const;
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
    
    void setExt(const double &a00, const double &a01, const double &a02, const double &a10, const double &a11, const double &a12 ){
      m_M[0][0] = a00, m_M[0][1] = a01, m_M[0][2] = a02;
      m_M[1][0] = a10, m_M[1][1] = a11, m_M[1][2] = a12;
    }
    
    void setExtInv(const double &a00, const double &a01, const double &a02, const double &a10, const double &a11, const double &a12 ){
      m_Minv[0][0] = a00, m_Minv[0][1] = a01, m_Minv[0][2] = a02;
      m_Minv[1][0] = a10, m_Minv[1][1] = a11, m_Minv[1][2] = a12;
    }
    
    /**
     * Transform a world coordinate into cell a index. 
     * @param wx source value x in world coordinates 
     * @param wy source value x in world coordinates  
     * @param cx des value x in pixel cell coordinates 
     * @param cx des value x in pixel cell coordinates 
     **/
    void world2Cell (const double wx, const double wy, int &cx, int &cy ) const;
    /**
     * Transform a cell a index into a world coordinate 
     * @param cx src value x in pixel cell coordinates 
     * @param cx src value x in pixel cell coordinates 
     * @param wx des value x in world coordinates 
     * @param wy des value x in world coordinates  
     **/
    void cell2World (const int cx, const int cy, double &wx, double &wy ) const;
    /// Transform a coordinate values into cell index. @param x metric value, @param y metric value  @return cell index
    cv::Point cvCellPoint ( double x, double y ) const;
    /// Transform a coordinate values into cell index. @param p metric value  @return cell index
    cv::Point cvCellPoint ( const cv::Point &p ) const;
    /// Transform a coordinate values into cell index. @param src metric value @param des cell indexes   @return cell indexes
    std::vector<cv::Point> cvCellPoint ( const std::vector<cv::Point> &src, std::vector<cv::Point> &des ) const;
    /// Transform a coordinate values into cell index. @param p metric value  @return cell index
    cv::Point cvCellPoint ( const cv::Point2f &p ) const;
    /// Transform a coordinate values into cell index. @param src metric value @param des cell indexes   @return cell indexes
    std::vector<cv::Point> cvCellPoint ( const std::vector<cv::Point2f> &src, std::vector<cv::Point> &des ) const;
    /// Transform a coordinate values into cell index. @param p metric value  @return cell index
    cv::Point cvCellPoint ( const cv::Point2d &p ) const;
    /// Transform a coordinate values into cell index. @param src metric value @param des cell indexes   @return cell indexes
    std::vector<cv::Point> cvCellPoint ( const std::vector<cv::Point2d> &src, std::vector<cv::Point> &des ) const;
    /// Transform a cell index into a coordinate value. @param x cell index @param y cell index  @return metric value
    cv::Point2d cvCoordinatePoint ( int &x, int &y ) const;
    /// Transform a cell index into a coordinate value. @param p cell index  @return metric value
    cv::Point2d cvCoordinatePoint ( const cv::Point &p ) const;
    static cv::Scalar cvGreen();
    static cv::Scalar cvBlue();
    static cv::Scalar cvRed();

    /** Returns transformation matrix
      */
    cv::Mat_<double> getTransformation ( ) const;
    /** Compares the entry type
     * @return true if the type T1 is equal to T
     */
    template <typename T1>
    bool isType() const {
        size_t type_hash_code = ( typeid ( T1 ).hash_code() );
        return ( m_type_hash_code == type_hash_code );
    }
    /** 
     * Compares resolution
     * @return true on same resolution
     */
    bool compareResolution ( const GridMapHeader& o ) const ;
    /** 
     * Compares grid size
     * @return true on same resolution
     */
    bool compareGridSize ( const GridMapHeader& o ) const ;
    /** 
     * Compares representation x_min, x_max, y_min and y_max;
     * @return true on same resolution
     */
    bool compareMetricRepresentation ( const GridMapHeader& o ) const ;

    /// pipes the header information
    friend std::ostream& operator<< ( std::ostream &output, const GridMapHeader &o ) {
        char msg[0xFF];
        sprintf ( msg, "[%zu, %zu] @ [%4.3f, %4.3f]m/px of %zu bytes, range x:  %4.3f -> %4.3f, y: %4.3f -> %4.3f, phi: %4.3frad, layer %zu, hash_code: %zu",
                  o.getSizeX(), o.getSizeY(), o.getResolutionX(), o.getResolutionY(),
                  o.getDepth (),
                  o.getXMin(), o.getXMax(),
                  o.getYMin(), o.getYMax(),
                  o.getRotation(),
                  o.getLayers(),
                  o.getTypeHashCode() );
        output << msg;
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, GridMapHeader &o ) {
        return input;
    }

    /// stream the header in a matlab format
    std::ostream& matlab ( std::ostream &output, const std::string &variable ) const {
        output << variable << ".x              = [ ";
        for ( size_t cx = 0; cx < m_size_x; cx++ ) {
            output << idx2x ( cx ) << ( cx < m_size_x-1?", ":"]\n" );
        }
        output << variable << ".y              = [ ";
        for ( size_t cy = 0; cy < m_size_y; cy++ ) {
            output << idx2y ( cy ) << ( cy < m_size_y-1?", ":"]\n" );
        }
        output << variable << ".x_min          =" <<  m_x_min << ",\n";
        output << variable << ".x_max          =" <<  m_x_max << ",\n";
        output << variable << ".y_min          =" <<  m_y_min << ",\n";
        output << variable << ".y_max          =" <<  m_y_max << ",\n";
        output << variable << ".x_resolution   =" <<  m_x_resolution << ",\n";
        output << variable << ".y_resolution   =" <<  m_y_resolution << ",\n";
        output << variable << ".rotation       =" <<  m_rotation << ",\n";
        output << variable << ".size_x         =" <<  m_size_x << ",\n";
        output << variable << ".size_y         =" <<  m_size_y << ",\n";
        output << variable << ".depth          =" <<  m_depth << ",\n";
        output << variable << ".layers         =" <<  m_layers << ",\n";
        output << variable << ".size           =" <<  m_size << ",\n";
        output << variable << ".size_total     =" <<  m_size_total << ",\n";
        output << variable << ".bytes          =" <<  m_bytes << ",\n";
        output << variable << ".bytes_total    =" <<  m_bytes_total << ",\n";
        output << variable << ".type_hash_code =" <<  m_type_hash_code << ",\n";
        return output;
    }
};
};


#endif //SHARED_MEM_OBJECT_GRID_MAP_HEADER_H





