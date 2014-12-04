/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
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
#ifndef SHARED_MEM_OBJECTS_POINT2D_H
#define SHARED_MEM_OBJECTS_POINT2D_H

#include <stdio.h>
#include <math.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/binary_object.hpp>
#include <shmfw/objects/base_object.h>
#include <shmfw/objects/vector2.h>
#include <boost/algorithm/string.hpp>
#include <opencv/cv.h>

namespace ShmFw {
class Point2D : public cv::Point_<double> {
public:
    Point2D(): cv::Point_<double>(){};
    Point2D(const Point2D &p): cv::Point_<double>(p) {};
    Point2D(const cv::Vec<double, 2> &v): cv::Point_<double>(v) {};
    Point2D(double x, double y): cv::Point_<double>(x, y) {};
    std::string getToStringFormat(const std::string &format) const{
      char buf[0xFF];
      sprintf(buf, format.c_str(), x, y); 
      return std::string(buf);
    }
    std::string getToString() const{
      return getToStringFormat("[ %lf, %lf]");
    }
    bool setFromString(const std::string &str) {
      int start = str.find("[");
      int end = str.find_last_of("]");
      std::string data = str.substr(start+1, end-1); 
      boost::erase_all(data, " ");
      if(sscanf(data.c_str(), "%lf,%lf", &x, &y) == EOF) return false; 
      return true;
    }
    friend std::ostream& operator<< (std::ostream &output, const Point2D &o) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>>(std::istream &input, Point2D &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    bool operator == ( const Point2D& o ) const {
        return x == o.x && y == o.y;
    }
    /** casts to vector2
     * @return vector2<double> cast
     **/
    ShmFw::Vector2<double> &asVector () {
        return (ShmFw::Vector2<double>&) *this;;
    } 
    /** casts to vector2
     * @return vector2<double> cast
     **/
    const ShmFw::Vector2<double> &asVector () const {
        return (ShmFw::Vector2<double>&) *this;;
    }   
    /** compares with within tolerance
     * @param o 
     * @param tolerance 
     **/
    bool equal( const Point2D& o, double tolerance = 0.0001 ) const {
         return this->asVector().equal(o.asVector(), tolerance);
    }
    /** Copies data from an array
     * @param src data source
     **/
    template<typename T2>
    Point2D& copyFrom ( const T2 &src) {
        x = src.x, y = src.y;
        return *this;
    }
    /** Copies data to an array
     * @param des data target
     **/
    template<typename T2>
    void copyTo( T2 &des ) const {
        des.x = x, des.y = y;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "x", x );
        ar & make_nvp ( "y", y );
    }
};
};
#endif //SHARED_MEM_OBJECTS_POINT2D_H

