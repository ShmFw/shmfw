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

#include <iostream>
#include <stdlib.h>
#include <signal.h>


#include <shmfw/objects/image.h>
#include <opencv2/highgui/highgui.hpp>
#include <shmfw/allocator.h>
#include <boost/program_options.hpp>

struct Prarmeters {
    std::string file_to_load;
    int reload;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "file_to_load,f", po::value<std::string> ( &params.file_to_load )->default_value ( "" ), "load a image and places it into shared memory" )
    ( "reload,r", po::value< int > ( &params.reload )->default_value ( 100 ), "reload time in ms, 0 means reload on signal, -1 means relaod owns to load image into shm" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name )->default_value ( "ImageMemory" ), "shared variable name" );

    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;;
        exit ( 1 );
    }
    po::notify ( vm );

    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }

    return params;
}

bool loop_program = true;

void terminate ( int param ) {
    std::cout << "Program was trying to abort or terminate." << std::endl;
    loop_program = false;
}

int main ( int argc, char **argv ) {
    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );
    Prarmeters params = readArgs ( argc, argv );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    if ( !params.file_to_load.empty() ) {
        if ( !params.file_to_load.empty() ) {
            cv::Mat img = cv::imread ( params.file_to_load.c_str(), CV_LOAD_IMAGE_COLOR );
            ShmFw::Alloc<ShmFw::ImageShm> shmImg ( params.variable_name, shmHdl );
            shmImg->copyFrom ( img, ShmFw::IMAGE_ENCODING_BGR8 );
        }
    }
    if ( !params.variable_name.empty() ) {
        if ( shmHdl->findName ( params.variable_name ) == false ) {
            std::cout << "no shared variable with the name: " << params.variable_name << std::endl;
            exit ( 1 );
        }
        for ( unsigned int i = 0, timeoutCounter = 0; ( params.reload >= 0 ) && loop_program; i++ ) {
            ShmFw::Alloc<ShmFw::ImageShm> shmImg ( params.variable_name, shmHdl );
            if ( !shmImg.isType<ShmFw::Alloc<ShmFw::ImageShm> >() ) {
                std::cout << "The shared variable: " << params.variable_name << ", is not a image!" << std::endl;
                exit(1);
            }
            if ( i == 0 ) std::cout << shmImg << std::endl;

            if ( shmImg.timed_wait ( 1000 ) == false ) {
                std::cout << std::setw ( 4 ) << timeoutCounter++ << ": waited 1000 ms" << std::endl;
            }

            if ( params.reload == 0 ) {
                shmImg.wait();
            }

            cv::Mat img;
            shmImg->toCvMat ( img );
            cv::namedWindow ( params.variable_name.c_str(), CV_WINDOW_AUTOSIZE );

            cv::imshow ( params.variable_name.c_str(), img );

            if ( params.reload == 0 ) cv::waitKey ( 10 );
            else if ( cv::waitKey ( params.reload ) >= 0 ) params.reload = -1;
        }
    }
    exit ( 0 );
}
