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
#ifndef SHARED_MEM_OPENCV_BRIDGE_H
#define SHARED_MEM_OPENCV_BRIDGE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <shmfw/image.h>

#define CLIP_TO_UCHAR(x)  (( x > 0xFF ) ? 0xFF : ( x < 0x00) ? 0x00 : x )
#define YCr_TO_RED(Y, Cr)  (Y + Cr + (Cr >> 2) + (Cr >> 3) + (Cr >> 5))
#define YCr_TO_GREEN(Y, Cr, Cb)  (Y - ((Cb >> 2) + (Cb >> 4) + (Cb >> 5)) - ((Cr >> 1) + (Cr >> 3) + (Cr >> 4) + (Cr >> 5)))
#define YCb_TO_BLUE(Y, Cb) (Y + Cb + (Cb >> 1) + (Cb >> 2) + (Cb >> 6))

namespace ShmFw {
int cvType ( int encoding ) {
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
int getEncoding ( int type ) {
    switch ( type ) {
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



ShmFw::Image &construct (ShmFw::Image &img, const std::string &name, HandlerPtr &shmHdl, cv::Mat &m ) {
  int depth = m.elemSize() / m.channels();
  img.construct(name, shmHdl, m.cols, m.rows, m.channels(), depth, getEncoding(m.type()), true);
  return img;
}

cv::Mat toCvMat ( const ShmFw::Image &img ) {
    return cv::Mat ( img.height(), img.width(), cvType ( img.encoding() ), img.data() );
};
void convertYUV422toBGR8 ( int height, int width, unsigned char *src, unsigned char *des ) {
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
void convertYUV422toBGR8 ( const ShmFw::Image &img, cv::Mat &des ) {
    des.create ( img.height(), img.width(), CV_8UC3 );
    convertYUV422toBGR8 ( img.height(), img.width(), img.data(), des.data );
}
void convertYUV422toRGB8 ( int height, int width, unsigned char *src, unsigned char *des ) {
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
void convertYUV422toRGB8 ( const ShmFw::Image &img, cv::Mat &des ) {
    des.create ( img.height(), img.width(), CV_8UC3 );
    convertYUV422toRGB8 ( img.height(), img.width(), img.data(), des.data );
}
void convertYUVtoBGR8 ( int height, int width, unsigned char *src, unsigned char *des ) {
    int Y, U, V, R, G, B;
    int Cr, Cb;
    for ( unsigned int i = height * width; i > 0; i-- ) {
        Y = *src++, U = *src++, V = *src++;
        Cb = V - 128, Cr = U - 128;

        R = YCr_TO_RED ( Y, Cr ), G = YCr_TO_GREEN ( Y, Cr, Cb ), B = YCb_TO_BLUE ( Y, Cb );
        *des++ = CLIP_TO_UCHAR ( B ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( R );
    }
}
void convertYUVtoBGR8 ( const ShmFw::Image &img, cv::Mat &des ) {
    des.create ( img.height(), img.width(), CV_8UC3 );
    convertYUVtoBGR8 ( img.height(), img.width(), img.data(), des.data );
}
void convertYUVtoRGB8 ( int height, int width, unsigned char *src, unsigned char *des ) {
    int Y, U, V, R, G, B;
    int Cr, Cb;
    for ( unsigned int i = height * width; i > 0; i-- ) {
        Y = *src++, U = *src++, V = *src++;
        Cb = V - 128, Cr = U - 128;
        R = YCr_TO_RED ( Y, Cr ), G = YCr_TO_GREEN ( Y, Cr, Cb ), B = YCb_TO_BLUE ( Y, Cb );
        *des++ = CLIP_TO_UCHAR ( R ), *des++ = CLIP_TO_UCHAR ( G ), *des++ = CLIP_TO_UCHAR ( B );
    }
}
void convertYUVtoRGB8 ( const ShmFw::Image &img, cv::Mat &des ) {
    des.create ( img.height(), img.width(), CV_8UC3 );
    convertYUVtoRGB8 ( img.height(), img.width(), img.data(), des.data );
}
void convertMONO32FtoMONO8 ( const ShmFw::Image &img, cv::Mat &des ) {
    double minVal, maxVal;
    cv::Mat tmp = toCvMat ( img );
    cv::minMaxLoc ( tmp, &minVal, &maxVal ); //find minimum and maximum intensities
    tmp.convertTo ( des, CV_8U, 255.0/ ( maxVal - minVal ), -minVal );
}

cv::Mat convertTo ( const ShmFw::Image &img, cv::Mat &m, int encoding ) {
    cv::Mat tmp = toCvMat ( img );
    switch ( encoding ) {
    case Image::MONO8:
        switch ( img.encoding() ) {
        case Image::MONO32F:
            convertMONO32FtoMONO8 ( img, m );
            break;
        default:
            std::cerr << "Error at:"  << __FILE__  << ", " << __FUNCTION__ << "unsuppored format";
            exit ( 1 );
        }
        break;
    case Image::BGR8:
        switch ( img.encoding() ) {
        case Image::BGR8:
            m = tmp; /// Do nothing
            break;
        case Image::RGB8:
            cv::cvtColor ( tmp, m, CV_BGR2RGB );
            break;
        case Image::YUV8:
            convertYUVtoBGR8 ( img, m );
            break;
        case Image::YUV422:
            convertYUV422toBGR8 ( img, m );
            break;
        default:
            std::cerr << "Error at:"  << __FILE__  << ", " << __FUNCTION__ << "unsuppored format";
            exit ( 1 );
        }
        break;
    default:
        std::cerr << "Error at:"  << __FILE__  << ", " << __FUNCTION__ << "unsuppored format";
        exit ( 1 );
    }
    return m;
}
};
#endif //SHARED_MEM_OPENCV_BRIDGE_H
