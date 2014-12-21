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


#ifndef SHMFW_LINESEGMENT2D_H
#define SHMFW_LINESEGMENT2D_H

#include <shmfw/objects/line2d.h>

namespace ShmFw {

template <typename T>
class LineSegment2D : private Line2D<T>{
protected:
    cv::Point_<T> p1_, p2_;
public:
    LineSegment2D() {};
    LineSegment2D(const LineSegment2D &l) 
    : Line2D<T>(l.line())
    , p1_(l.p1())
    , p2_(l.p2()) {
    };
    LineSegment2D(cv::Vec<T,4> &v) 
    : Line2D<T>(v[0], v[1], v[2], v[3], true)
    , p1_(v[0], v[1]) 
    , p2_(v[2], v[3]) {
    };
    LineSegment2D(const cv::Point_<T> &pt1, const cv::Point_<T> &pt2)
    : Line2D<T>(pt1, pt2, true)
    , p1_(pt1)
    , p2_(pt2) {
    };
    LineSegment2D(const T &x1, const T &y1, const T &x2, const T &y2)
    : Line2D<T>(x1, y1, x2, y2, true)
    , p1_(x1, y1)
    , p2_(x2, y2) {
    };
    const T &x1() const {
        return  p1_.x;
    }
    const T &y1() const {
        return  p1_.y;
    }
    const T &x2() const {
        return p2_.x;
    }
    const T &y2() const {
        return p2_.y;
    }
    const cv::Point_<T> &p1() const {
        return p1_;
    }
    const cv::Point_<T> &p2() const {
        return p2_;
    }
    const Line2D<T> &line() const {
        return *this;
    }
    const T length() const {
        return cv::norm(cv::Vec<T,2> (p2_.x - p1_.x, p2_.y-p1_.y));
    }    
    /// comparison operator @return true on equal
    bool operator == ( const LineSegment2D<T>& o ) const {
        return p1() == o.p1() && p2() == o.p2();
    }
    template <typename T2>
    LineSegment2D<T> set(const T2 &x1, const T2 &y1, const T2 &x2, const T2 &y2) {
        Line2D<T>::set(x1, y1, x2, y2, true);
        p1_.x = x1, p1_.y = y1, p2_.x = x2, p2_.y = y2;
        return *this;
    }
    template <typename T2>
    LineSegment2D<T> set(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2) {
        set(pt1.x, pt1.y, pt2.x, pt2.y);
        return *this;
    }
    /** computes distance to line segment 
     * @see http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * @return distance to line between the segment endpoints or the distance to the nearest endpoints **/
    template <typename T2>
    T distanceTo(const cv::Point_<T2> &pt) const {
        T px = x2()-x1();
        T py = y2()-y1();
        T l2 = px*px + py*py;
        T u =  ((pt.x - x1()) * px + (pt.y - y1()) * py) / l2;
        if (u > 1) u = 1;
        else if (u < 0) u = 0;
        T x = x1() + u * px;
        T y = y1() + u * py;
        T dx = x - pt.x;
        T dy = y - pt.y;
        T dist = sqrt(dx*dx + dy*dy);
        return dist;
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::LineSegment2D<T> &o ) {
        os << "[[" << o.x1() <<  ", " << o.y1() << "], [" << o.x2() << ", " << o.y2() << "] ]";
        return os;
    };
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "x1", p1_.x);
        ar & make_nvp ( "y1", p1_.y);
        ar & make_nvp ( "x2", p2_.x);
        ar & make_nvp ( "y2", p2_.y);
        if ( archive::is_loading::value ) {
          this->normalize();
        }
    }
};
typedef LineSegment2D<float> LineSegment2Df;
typedef LineSegment2D<double> LineSegment2Dd;

};
#endif // SHMFW_LINESEGMENT2D_H
