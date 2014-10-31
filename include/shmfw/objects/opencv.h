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

#ifndef SHARED_MEM_OBJECTS_OPENCV_H
#define SHARED_MEM_OBJECTS_OPENCV_H

#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>

namespace cv{
inline int solveCvType(const std::string &cvtype){  
    int type = -1;
    if ( boost::iequals ( "CV_8U", cvtype ) ) type = CV_8U;
    else if ( boost::iequals ( "CV_8S", cvtype ) ) type = CV_8S;
    else if ( boost::iequals ( "CV_16U", cvtype ) ) type = CV_16U;
    else if ( boost::iequals ( "CV_16S", cvtype ) ) type = CV_16S;
    else if ( boost::iequals ( "CV_32F", cvtype ) ) type = CV_32F;
    else if ( boost::iequals ( "CV_64F", cvtype ) ) type = CV_64F;
    else if ( boost::iequals ( "CV_8UC1", cvtype ) ) type = CV_8UC1;
    else if ( boost::iequals ( "CV_8UC2", cvtype ) ) type = CV_8UC2;
    else if ( boost::iequals ( "CV_8UC3", cvtype ) ) type = CV_8UC3;
    else if ( boost::iequals ( "CV_8UC4", cvtype ) ) type = CV_8UC4;
    else if ( boost::iequals ( "CV_8SC1", cvtype ) ) type = CV_8SC1;
    else if ( boost::iequals ( "CV_8SC2", cvtype ) ) type = CV_8SC2;
    else if ( boost::iequals ( "CV_8SC3", cvtype ) ) type = CV_8SC3;
    else if ( boost::iequals ( "CV_8SC4", cvtype ) ) type = CV_8SC4;
    else if ( boost::iequals ( "CV_16UC1", cvtype ) ) type = CV_16UC1;
    else if ( boost::iequals ( "CV_16UC2", cvtype ) ) type = CV_16UC2;
    else if ( boost::iequals ( "CV_16UC3", cvtype ) ) type = CV_16UC3;
    else if ( boost::iequals ( "CV_16UC4", cvtype ) ) type = CV_16UC4;
    else if ( boost::iequals ( "CV_16SC1", cvtype ) ) type = CV_16SC1;
    else if ( boost::iequals ( "CV_16SC2", cvtype ) ) type = CV_16SC2;
    else if ( boost::iequals ( "CV_16SC3", cvtype ) ) type = CV_16SC3;
    else if ( boost::iequals ( "CV_16SC4", cvtype ) ) type = CV_16SC4;
    else if ( boost::iequals ( "CV_32SC1", cvtype ) ) type = CV_32SC1;
    else if ( boost::iequals ( "CV_32SC2", cvtype ) ) type = CV_32SC2;
    else if ( boost::iequals ( "CV_32SC3", cvtype ) ) type = CV_32SC3;
    else if ( boost::iequals ( "CV_32SC4", cvtype ) ) type = CV_32SC4;
    else if ( boost::iequals ( "CV_32FC1", cvtype ) ) type = CV_32FC1;
    else if ( boost::iequals ( "CV_32FC2", cvtype ) ) type = CV_32FC2;
    else if ( boost::iequals ( "CV_32FC3", cvtype ) ) type = CV_32FC3;
    else if ( boost::iequals ( "CV_32FC4", cvtype ) ) type = CV_32FC4;
    else if ( boost::iequals ( "CV_64FC1", cvtype ) ) type = CV_64FC1;
    else if ( boost::iequals ( "CV_64FC2", cvtype ) ) type = CV_64FC2;
    else if ( boost::iequals ( "CV_64FC3", cvtype ) ) type = CV_64FC3;
    else if ( boost::iequals ( "CV_64FC4", cvtype ) ) type = CV_64FC4;
    return type;
}
  
inline bool operator== ( const RotatedRect &a, const RotatedRect &b ) {
    return ((a.center == b.center) && (a.size == b.size) && (a.angle == b.angle));
}
inline bool operator!= ( const RotatedRect &a, const RotatedRect &b ) {
    return ((a.center != b.center) && (a.size != b.size) && (a.angle != b.angle));
}
};

#endif //SHARED_MEM_OBJECTS_OPENCV_H

