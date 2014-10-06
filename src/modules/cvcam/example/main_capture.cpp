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

#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

void help() {
    std::cout << "\nThis is a demo that shows an image from camer or file\n";
}

int main ( int argc, char *argv[] ) {

    cv::VideoCapture cap;
    cv::Mat imgSrc;
    cv::Mat imgGray;
    if ( argc != 2 ) {
        cap.open ( 0 );
        if ( !cap.isOpened() ) {
            std::cout << "***Could not initialize capturing...***\n";
            help();
            return -1;
        }
        cap.set ( CV_CAP_PROP_FRAME_WIDTH, 800 );
        cap.set ( CV_CAP_PROP_FRAME_HEIGHT, 600 );
        std::cout << "Video " <<
                  ": width=" << cap.get ( CV_CAP_PROP_FRAME_WIDTH ) <<
                  ", height=" << cap.get ( CV_CAP_PROP_FRAME_HEIGHT ) <<
                  ", nframes=" << cap.get ( CV_CAP_PROP_FRAME_COUNT ) << std::endl;
    }
    if ( !cap.isOpened() ) {
        printf ( "%s\n", argv[1] );
        imgSrc = cv::imread ( argv[1] );
    } else {
        cap >> imgSrc;
    }
    if ( imgSrc.empty() ) {
        std::cout << "***image...***\n";
        return -1;
    }
    cv::cvtColor ( imgSrc, imgGray, CV_RGB2GRAY );

    cv::namedWindow ( "image",1 );
    do {
        if ( cap.isOpened() ) {
            cap >> imgSrc;
            cv::cvtColor ( imgSrc, imgGray, CV_RGB2GRAY );
        }
        if ( imgSrc.empty() ) {
            continue;
        }
        cv::imshow ( "image", imgSrc );
    } while ( cv::waitKey ( 30 ) < 0 ) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

