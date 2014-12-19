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
#include <signal.h>
#include <boost/program_options.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <shmfw/objects/image.h>
#include <shmfw/allocator.h>

#include "shmfw/log.h"


struct Prarmeters {
    bool clear;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    std::string configFile;
    std::string image_file;
    int device;
    int height;
    int width;
    bool imshow;
};



int program_options ( int argc, char* argv[], Prarmeters &params ) {
    boost::program_options::options_description desc ( "Allowed Parameters" );
    desc.add_options() //
    ( "help", "get this help message" ) //
    ( "cfg", boost::program_options::value<std::string> ( &params.configFile ), "Config file" ) //
    ( "imshow,s", "opens a highgui window and shows the images" ) //
    ( "device,d", boost::program_options::value<int> ( &params.device )->default_value ( 0 ), "device" )
    ( "image_file,i", boost::program_options::value<std::string> ( &params.image_file )->default_value ( "" ), "image used instead of an device" )
    ( "width,w", boost::program_options::value<int> ( &params.width )->default_value ( 0 ), "image width, zero means auto" )
    ( "height,h", boost::program_options::value<int> ( &params.height )->default_value ( 0 ), "image height, zero means auto" )
    ( "shm_memory_name", boost::program_options::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size", boost::program_options::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" )
    ( "variable_name,v", boost::program_options::value<std::string > ( &params.variable_name )->default_value ( "cvcam" ), "shared variable" );


    std::cout << std::endl;

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store ( boost::program_options::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception& ex ) {
        std::cout << desc << "\n";
        return 1;
    }
    boost::program_options::notify ( vm );

    if ( vm.count ( "help" ) ) {
        std::cout << desc << "\n";
        exit ( 0 );
    }

    if ( !params.configFile.empty() ) {
        std::ifstream file ( params.configFile.c_str(), std::ifstream::in );
        if ( file.is_open() ) {
            try {
                boost::program_options::store ( boost::program_options::parse_config_file ( file, desc ), vm );
                boost::program_options::notify ( vm );
            } catch ( const std::exception& ex ) {
                std::cout << "Error reading config file: " << ex.what() << std::endl;
                exit ( 0 );
            }
        } else {
            std::cout << "Error opening config file " << params.configFile << std::endl;
            exit ( 0 );
        }
    }
    params.imshow = ( vm.count ( "imshow" ) > 0 );
    return 0;
}

bool loop_program = true;

void terminate ( int param ) {
    std::cout << "Program was trying to abort or terminate." << std::endl;
    loop_program = false;
}

int main ( int argc, char *argv[] ) {

    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );
    Prarmeters params;
    program_options ( argc, argv, params );

    cv::Point x;

    cv::VideoCapture cap;
    cv::Mat imgSrc;
    if ( params.image_file.empty() ) {
        cap.open ( params.device );
        if ( !cap.isOpened() ) {
            std::cout << "***Could not initialize capturing...***\n";
            exit ( 0 );
        }
        if ( params.width > 0 ) cap.set ( CV_CAP_PROP_FRAME_WIDTH, params.width );
        if ( params.height > 0 ) cap.set ( CV_CAP_PROP_FRAME_HEIGHT, params.height );
        std::cout << "Video " <<
                  ": width=" << cap.get ( CV_CAP_PROP_FRAME_WIDTH ) <<
                  ", height=" << cap.get ( CV_CAP_PROP_FRAME_HEIGHT ) << std::endl;
        if ( cap.isOpened() ) {
            cap >> imgSrc;
        } else {
            std::cout << "***Could not get any image...***\n";
            exit ( 0 );
        }
    } else {
        std::cout << "using file: " << params.image_file << std::endl;
        imgSrc = cv::imread ( params.image_file );
    }
    if ( imgSrc.empty() ) {
        std::cout << "***image...***\n";
        return -1;
    }

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Alloc<ShmFw::ImageShm> shmImg ( params.variable_name, shmHdl );
    shmImg.unlock();
    cv::Mat shmMat;
    shmImg->copyFrom(imgSrc);
    shmImg->cvMat ( shmMat );
    std::cout << "You should be able to view the image with:  ";
    std::cout << "shmfw-image_view -v " << params.variable_name << std::endl;
    do {
        shmImg.lock();
        if ( cap.isOpened() ) {
            cap >> shmMat;
        }
        if ( shmMat.empty() ) {
            continue;
        }
        shmImg.unlock();
        shmImg.itHasChanged();
        if ( params.imshow ) {
            cv::imshow ( "image", shmMat );
        }
    } while ( ( cv::waitKey ( 30 ) < 0 ) && loop_program ) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

