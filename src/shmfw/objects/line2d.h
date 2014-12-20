/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef SHMFW_LINE2D_H
#define SHMFW_LINE2D_H

#include <opencv2/core/core.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <cstdio>

namespace ShmFw {

template <typename T>
class Line2D : public cv::Vec<T,3> {
public:
    Line2D(){
    };
    Line2D(const Line2D &l) : cv::Vec<T,3>(l){
    };
    Line2D(cv::Vec<T,3> &l, bool normalize = true)
            : cv::Vec<T,3>(l) {
        if (normalize) this->normalize();
    };
    template <typename T2>
    Line2D(const T2 &x1, const T2 &y1, const T2 &x2, const T2 &y2, bool normalize = true) {
        set(x1,y1,x2,y2, normalize);
    }
    template <typename T2>
    Line2D(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2, bool normalize = true) {
        set(pt1, pt2, normalize);
    }
    template <typename T2>
    Line2D(const cv::Point3_<T2> &pt1, const cv::Point3_<T2> &pt2, bool normalize = true) {
        set(pt1, pt2, normalize);
    }
    template <typename T2, int cn>
    Line2D(const cv::Vec<T2, cn> &v1, const cv::Vec<T2, cn> &v2, bool normalize = true) {
        set(v1, v2, normalize);
    }
    T &A() {
        return this->val[0];
    }
    const T &A() const {
        return this->val[0];
    }
    T &B() {
        return this->val[1];
    }
    const T &B() const {
        return this->val[1];
    }
    T &C() {
        return this->val[2];
    }
    const T &C() const {
        return this->val[2];
    }
    void normalize() {
        double r = sqrt(this->val[0]*this->val[0] + this->val[1]*this->val[1]);
        this->val[0] /= r, this->val[1] /= r, this->val[2] /= r;
    }
    /** @pre normalize */
    template <typename T2>
    T distanceTo(const cv::Point_<T2> &p)  const {
        return this->val[0]*((T)p.x) + this->val[1]*((T)p.y) + this->val[2];
    }
    /** @pre normalize */
    template <typename T2>
    cv::Point_<T> pointOnLine(const cv::Point_<T2> &p) {
        T d = distanceTo(p);
        return cv::Point_<T>(p.x - d * A(), p.y - d * B());
    }
    cv::Point_<T> intersection(const Line2D<T> &l) const{
        cv::Vec<T,3> h = this->cross(l);
        return cv::Point_<T>(h[0]/h[2],h[1]/h[2]);
    }
    cv::Vec<T,2> normal() {
        return cv::Vec<T,2>(this->val[0], this->val[1]);
    }
    template <typename T2>
    Line2D<T> set(const T2 &x1, const T2 &y1, const T2 &x2, const T2 &y2, bool normalize = true) {
        this->val[0] = y1-y2, this->val[1] = x2-x1, this->val[2] = x1*y2-y1*x2; /// cross product with homogenios vectors
        if (normalize) this->normalize();
        return *this;
    }
    template <typename T2>
    Line2D<T> set(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2, bool normalize = true) {
       set(pt1.x, pt1.y, pt2.x, pt2.y, normalize);
        return *this;
    }
    template <typename T2>
    Line2D<T> set(const cv::Point3_<T2> &pt1, const cv::Point3_<T2> &pt2, bool normalize = true) {
       set(pt1.x, pt1.y, pt2.x, pt2.y, normalize);
        return *this;
    }
    template <typename T2, int cn>
    Line2D<T> set(const cv::Vec<T2, cn> &v1, const cv::Vec<T2, cn> &v2, bool normalize = true) {
       set(v1[0], v1[1], v2[0], v2[1], normalize);
        return *this;
    }
    template <typename T2>
    Line2D<T> &set(const cv::Matx<T2, 3, 1> &v) {
        this->val[0] = v.val[0], this->val[1] = v.val[1], this->val[2] = v.val[2];
        return *this;
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::Line2D<T> &o ) {
        os << "[" << o.A() <<  ", " << o.B() << ", " << o.C() << "]";
        return os;
    };
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "val", this->val );
    }
};
typedef Line2D<float> Line2Df;
typedef Line2D<double> Line2Dd;

};
#endif // SHMFW_LINE2D_H
