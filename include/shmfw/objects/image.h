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

#ifndef SHARED_MEM_OBJECT_IMAGE_H
#define SHARED_MEM_OBJECT_IMAGE_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <shmfw/objects/point.h>
#include <shmfw/handler.h>
#include <opencv/cv.h>

#define CLIP_TO_UCHAR(x)  (( x > 0xFF ) ? 0xFF : ( x < 0x00) ? 0x00 : x )
#define YCr_TO_RED(Y, Cr)  (Y + Cr + (Cr >> 2) + (Cr >> 3) + (Cr >> 5))
#define YCr_TO_GREEN(Y, Cr, Cb)  (Y - ((Cb >> 2) + (Cb >> 4) + (Cb >> 5)) - ((Cr >> 1) + (Cr >> 3) + (Cr >> 4) + (Cr >> 5)))
#define YCb_TO_BLUE(Y, Cb) (Y + Cb + (Cb >> 1) + (Cb >> 2) + (Cb >> 6))

namespace ShmFw {

/**
 * @note this class can only be used in combination with ShmFw::Alloc
 **/
namespace bi = boost::interprocess;
class Image {
    typedef bi::allocator<uint8_t, SegmentManager> Allocator;
    typedef bi::vector<uint8_t, Allocator > VectorData;

public:
    enum image_encodings {
        NA = 0,
        MONO8 = 0x10, MONO16, MONO32F, MONO64F,
        RGB8 = 0x20, RGBA8, RGB16, RGBA16,
        BGR8 = 0x30, BGRA8, BGR16, BGRA16,
        YUV8 = 0x40, YUV16, YUV422
    };
    int encoding;        /// encoding
    int width;           /// width = columns
    int height;          /// height = rows
    int depth;           /// depth  in bytes
    int channels;        /// channels  in bytes
    int widthStep;       /// Size of aligned image row in bytes.  width*channels*depth
    int pixelStep;       /// Size of aligned pixel row in bytes.  channels*depth
    VectorData data;

    void updateStepValues() {
        pixelStep =  channels * depth;
        widthStep = width * pixelStep;
    }

    Image ( const VoidAllocator &void_alloc )
        : data ( void_alloc )
    {}

    Image ( const Image &p )
        : data ( p.data.get_allocator() ) {
        copyFrom ( p );
    }

    std::string getToString() const {
        std::stringstream ss;
        return ss.str();
    }
    void getFromString ( const std::string &str ) {
    }
    friend std::ostream& operator<< ( std::ostream &output, const Image &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Image &o ) {
        return input;
    }
    bool operator == ( const Image& o ) const {
        return true;
    }
    Image &operator = ( const Image& o ) {
        copyFrom ( o );
        return *this;
    }
    void copyTo ( Image& des ) const {
        des.encoding = encoding;
        des.width = width;
        des.height = height;
        des.depth = depth;
        des.channels = channels;
        des.widthStep = widthStep;
        des.pixelStep = pixelStep;
        des.data.resize ( data.size() );
        for ( size_t i = 0; i < des.data.size(); i++ ) {
            des.data[i] = data[i];
        }
    }
    Image& copyFrom ( const Image& src ) {
        encoding = src.encoding;
        width = src.width;
        height = src.height;
        depth = src.depth;
        channels = src.channels;
        widthStep = src.widthStep;
        pixelStep = src.pixelStep;
        data.resize ( src.data.size() );
        for ( size_t i = 0; i < src.data.size(); i++ ) {
            data[i] = src.data[i];
        }
        return *this;
    }
    void copyTo ( cv::Mat& des ) const {
        switch ( encoding ) {
        case MONO8:
            des.create ( height, width, CV_8U );
            break;
        case MONO16:
            des.create ( height, width, CV_16U );
            break;
        case MONO32F:
            des.create ( height, width, CV_32F );
            break;
        case MONO64F:
            des.create ( height, width, CV_64F );
            break;
        case RGB8:
        case BGR8:
        case YUV8:
            des.create ( height, width, CV_8UC3 );
            break;
        case RGBA8:
        case BGRA8:
            des.create ( height, width, CV_8UC4 );
            break;
        case YUV16:
            des.create ( height, width, CV_16UC3 );
            break;
        case RGBA16:
        case BGRA16:
            des.create ( height, width, CV_16UC4 );
            break;
        }
        size_t s = height * widthStep;
        for ( size_t i = 0; i < s; i++ ) {
            des.data[i] = data[i];
        }
    }
    Image& copyFrom ( const cv::Mat& src ) {
        encoding = getEncoding ( src.type() );
        width = src.cols;
        height = src.rows;
        depth = src.elemSize() / src.channels();
        channels = src.channels();
        updateStepValues();
        size_t s = height * widthStep;
        data.resize ( s );
        for ( size_t i = 0;  i < s; i++ ) {
            data[i] = src.data[i];
        }
        return *this;
    }
    Image& copyFrom ( const cv::Mat& src, image_encodings encoding ) {
        this->encoding = encoding;
        width = src.cols;
        height = src.rows;
        depth = src.elemSize() / src.channels();
        channels = src.channels();
        updateStepValues();
        size_t s = height * widthStep;
        data.resize ( s );
        for ( size_t i = 0;  i < s; i++ ) {
            data[i] = src.data[i];
        }
        return *this;
    }
    static int cvType ( int encoding ) {
        switch ( encoding ) {
        case ShmFw::Image::NA:
        case ShmFw::Image::MONO8:
            return CV_8UC1;
        case ShmFw::Image::MONO16:
            return CV_16UC1;
        case ShmFw::Image::MONO32F:
            return CV_32FC1;
        case ShmFw::Image::MONO64F:
            return CV_64FC1;
        case ShmFw::Image::RGB8:
        case ShmFw::Image::BGR8:
        case ShmFw::Image::YUV8:
            return CV_8UC3;
        case ShmFw::Image::RGB16:
        case ShmFw::Image::BGR16:
        case ShmFw::Image::YUV16:
            return CV_16UC3;
        case ShmFw::Image::RGBA8:
        case ShmFw::Image::BGRA8:
            return CV_8UC4;
        case ShmFw::Image::RGBA16:
        case ShmFw::Image::BGRA16:
            return CV_16UC4;
        case ShmFw::Image::YUV422:
            return CV_8UC2;
        default:
            return CV_8UC1;
        };
    };
    static int getEncoding ( int cv_type ) {
        switch ( cv_type ) {
        case CV_8UC1:
            return ShmFw::Image::MONO8;
        case CV_16UC1:
            return ShmFw::Image::MONO16;
        case CV_32FC1:
            return ShmFw::Image::MONO32F;
        case CV_64FC1:
            return ShmFw::Image::MONO64F;
        case CV_8UC3:
            return ShmFw::Image::BGR8;
        case CV_16UC3:
            return ShmFw::Image::BGR16;
        default:
            return ShmFw::Image::NA;
        };
    };

    cv::Mat &toCvMat ( cv::Mat &img ) {
        img = cv::Mat ( height, width, cvType ( encoding ), &data[0] );
        return img;
    };

    static void convertYUV422toBGR8 ( int height, int width, uint8_t *src, uint8_t *des ) {
        int Y1, Y2, U, V, R, G, B;
        int Cr, Cb;
        for ( unsigned int i = height * width / 2; i > 0; i-- ) {
            Y1 = src[0], U = src[1], Y2 = src[2], V = src[3], src += 4;
            Cb = V - 128, Cr = U - 128;

            R = YCr_TO_RED ( Y1, Cr ), G = YCr_TO_GREEN ( Y1, Cr, Cb ), B = YCb_TO_BLUE ( Y1, Cb );
            *des++ = CLIP_TO_UCHAR ( B ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( R );

            R = YCr_TO_RED ( Y2, Cr ), G = YCr_TO_GREEN ( Y2, Cr, Cb ), B = YCb_TO_BLUE ( Y2, Cb );
            *des++ = CLIP_TO_UCHAR ( B ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( R );
        }
    }
    static void convertYUV422toRGB8 ( int height, int width, uint8_t *src, uint8_t *des ) {
        int Y1, Y2, U, V, R, G, B;
        int Cr, Cb;
        for ( unsigned int i = height * width / 2; i > 0; i-- ) {
            Y1 = src[0], U = src[1], Y2 = src[2], V = src[3], src += 4;
            Cb = V - 128, Cr = U - 128;

            R = YCr_TO_RED ( Y1, Cr ), G = YCr_TO_GREEN ( Y1, Cr, Cb ), B = YCb_TO_BLUE ( Y1, Cb );
            *des++ = CLIP_TO_UCHAR ( B ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( R );

            R = YCr_TO_RED ( Y2, Cr ), G = YCr_TO_GREEN ( Y2, Cr, Cb ), B = YCb_TO_BLUE ( Y2, Cb );
            *des++ = CLIP_TO_UCHAR ( B ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( R );
        }
    }
    static void convertYUVtoBGR8 ( int height, int width, uint8_t *src, uint8_t *des ) {
        int Y, U, V, R, G, B;
        int Cr, Cb;
        for ( unsigned int i = height * width; i > 0; i-- ) {
            Y = *src++, U = *src++, V = *src++;
            Cb = V - 128, Cr = U - 128;

            R = YCr_TO_RED ( Y, Cr ), G = YCr_TO_GREEN ( Y, Cr, Cb ), B = YCb_TO_BLUE ( Y, Cb );
            *des++ = CLIP_TO_UCHAR ( B ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( R );
        }
    }
    static void convertYUVtoRGB8 ( int height, int width, uint8_t *src, uint8_t *des ) {
        int Y, U, V, R, G, B;
        int Cr, Cb;
        for ( unsigned int i = height * width; i > 0; i-- ) {
            Y = *src++, U = *src++, V = *src++;
            Cb = V - 128, Cr = U - 128;
            R = YCr_TO_RED ( Y, Cr ), G = YCr_TO_GREEN ( Y, Cr, Cb ), B = YCb_TO_BLUE ( Y, Cb );
            *des++ = CLIP_TO_UCHAR ( R ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( B );
        }
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "encoding", encoding );
        ar & make_nvp ( "width", width );
        ar & make_nvp ( "height", height );
        ar & make_nvp ( "depth", depth );
        ar & make_nvp ( "channels", channels );
        ar & make_nvp ( "widthStep", widthStep );
        ar & make_nvp ( "pixelStep", pixelStep );
        ar & make_nvp ( "data", data );
    }
};

};


#endif //SHARED_MEM_OBJECT_IMAGE_H


