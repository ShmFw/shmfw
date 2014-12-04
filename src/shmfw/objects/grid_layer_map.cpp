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


#include "grid_layer_map.h"

using namespace ShmFw;

size_t GridLayerMapHeader::getSizeX() const {
  return m_size_x;
}
size_t GridLayerMapHeader::getColumns() const {
  return getSizeX();
}
size_t GridLayerMapHeader::getSizeY() const {
  return m_size_y;
}
size_t GridLayerMapHeader::getRows() const {
  return getSizeY();
}
size_t GridLayerMapHeader::getDepth() const {
  return m_depth;
}
size_t GridLayerMapHeader::getLayers() const {
  return m_layers;
}
size_t GridLayerMapHeader::getTypeHashCode() const {
  return m_type_hash_code;
}
double GridLayerMapHeader::getXMin() const  {
  return m_x_min;
}
double GridLayerMapHeader::getXMax() const  {
  return m_x_max;
}
double GridLayerMapHeader::getYMin() const  {
  return m_y_min;
}
double GridLayerMapHeader::getYMax() const  {
  return m_y_max;
}
double GridLayerMapHeader::getResolutionX() const  {
  return m_x_resolution;
}
double GridLayerMapHeader::getResolutionY() const  {
  return m_y_resolution;
}
size_t GridLayerMapHeader::size() const {
  return m_size_x*m_size_y;
}

cv::Point GridLayerMapHeader::cvCellPoint (double x, double y ) const {
  return cv::Point ( x2idx ( x ), y2idx ( y ) );
}
cv::Point GridLayerMapHeader::cvCellPoint ( const cv::Point &p ) const {
  return cv::Point ( x2idx ( p.x ), y2idx ( p.y ) );
}
cv::Point GridLayerMapHeader::cvCellPoint ( const cv::Point2f &p ) const {
  return cv::Point ( x2idx ( p.x ), y2idx ( p.y ) );
}
cv::Point GridLayerMapHeader::cvCellPoint ( const cv::Point2d &p ) const {
  return cv::Point ( x2idx ( p.x ), y2idx ( p.y ) );
}
cv::Point2d GridLayerMapHeader::cvPosePoint ( int &x, int &y ) const {
  return cv::Point2d ( idx2x ( x ), idx2y ( y ) );
}
cv::Point2d GridLayerMapHeader::cvPosePoint ( const cv::Point &p ) const {
  return cv::Point2d ( idx2x ( p.x ), idx2y ( p.y ) );
}
int GridLayerMapHeader::cvtype() const {
  int type = -1;
  if ( m_depth == 1 ) type = CV_8U;
  else if ( m_depth == 2 ) type = CV_16U;
  else if ( m_depth == 3 ) type = CV_8UC3;
  return type;
}
void GridLayerMapHeader::setBounderies ( const double x_min, const double x_max, const double y_min, const double y_max, const double x_resolution, const double y_resolution ) {
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
void GridLayerMapHeader::setBounderies ( const double x_min, const double x_max, const double y_min, const double y_max, const size_t size_x, const size_t size_y ) {
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

int   GridLayerMapHeader::x2idx ( double x ) const {
  return static_cast<int> ( ( x-this->m_x_min ) /this->m_x_resolution );
}
int   GridLayerMapHeader::y2idx ( double y ) const {
  return static_cast<int> ( ( y-this->m_y_min ) /this->m_y_resolution );
}
int   GridLayerMapHeader::xy2idx ( double x,double y ) const {
  return x2idx ( x ) + y2idx ( y ) *this->m_size_x;
}
void  GridLayerMapHeader::idx2cxcy ( const int &idx,  int &cx, int &cy ) const {
  cx = idx % this->m_size_x;
  cy = idx / this->m_size_x;
}
double   GridLayerMapHeader::idx2x ( int cx ) const {
  return this->m_x_min+ ( cx+0.5f ) *this->m_x_resolution;
}
double   GridLayerMapHeader::idx2y ( int cy ) const {
  return this->m_y_min+ ( cy+0.5f ) *this->m_y_resolution;
}
int GridLayerMapHeader::x2idx ( double x,double x_min ) const {
  SHMFW_UNUSED_PARAM ( x_min );
  return static_cast<int> ( ( x-this->m_x_min ) / this->m_x_resolution );
}
int GridLayerMapHeader::y2idx ( double y, double y_min ) const {
  SHMFW_UNUSED_PARAM ( y_min );
  return static_cast<int> ( ( y-this->m_y_min ) /this->m_y_resolution );
}
std::ostream& operator<< ( std::ostream &output, const GridLayerMapHeader &o ) {
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
std::istream& operator>> ( std::istream &input, GridLayerMapHeader &o ) {
  return input;
}