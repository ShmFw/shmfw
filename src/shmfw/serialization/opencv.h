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


#ifndef SHMFW_SERIALIZATION_OPENCV_H
#define SHMFW_SERIALIZATION_OPENCV_H


#include <opencv2/core/core.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>

namespace boost {
namespace serialization {

/// serialize function
template<class Archive, class T> inline  void serialize ( Archive &ar, cv::Rect_<T> &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "x", o.x );
    ar & boost::serialization::make_nvp ( "x", o.y );
    ar & boost::serialization::make_nvp ( "width", o.width );
    ar & boost::serialization::make_nvp ( "height", o.height );
}

template<class Archive, class T> inline  void serialize ( Archive &ar, cv::Point_<T> &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "x", o.x );
    ar & boost::serialization::make_nvp ( "x", o.y );
}

template<class Archive, class T> inline  void serialize ( Archive &ar, cv::Point3_<T> &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "x", o.x );
    ar & boost::serialization::make_nvp ( "x", o.y );
    ar & boost::serialization::make_nvp ( "z", o.z );
}

template<class Archive, class T> inline  void serialize ( Archive &ar, cv::Size_<T> &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "width", o.width );
    ar & boost::serialization::make_nvp ( "height", o.height );
}

template<class Archive> inline  void serialize ( Archive &ar, cv::RotatedRect &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "center", o.center );
    ar & boost::serialization::make_nvp ( "size", o.size );
    ar & boost::serialization::make_nvp ( "angle", o.angle );
}

}; // namespace serialization
}; // namespace boost
#endif // SHMFW_SERIALIZATION_OPENCV_H
