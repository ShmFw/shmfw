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


#include <opencv/highgui.h>
#include <boost/program_options.hpp>
#include <shmfw/objects/image.h>
#include <shmfw/allocator.h>
#include <shmfw/serialization/io_file.h>

namespace bi = boost::interprocess;
namespace po = boost::program_options;

struct Prarmeters {
    std::string file_to_load;
    int reload;
    bool clear;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
};

Prarmeters readArgs ( int argc, char **argv ) {
    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "clear,c", "clears the shm first, default is no" )
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
    params.clear = ( vm.count ( "clear" ) > 0 );

    return params;
}

int main ( int argc, char **argv ) {
    Prarmeters params = readArgs ( argc, argv );
    if ( params.clear ) {
        ShmFw::Handler::removeSegment ( params.shm_memory_name );
        std::cout << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
    }
    if ( !params.file_to_load.empty() ) {
        ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
        if ( !params.file_to_load.empty() ) {
            cv::Mat img = cvLoadImage ( params.file_to_load.c_str(), 1 );
	    ShmFw::Alloc<ShmFw::Image> shm_image( params.variable_name, shmHdl );
	    shm_image->copyFrom(img);
	    cv::Mat des;
	    shm_image->toCvMat(des);
            cv::imshow ( params.variable_name.c_str(), des );
            cv::waitKey ( -1 );
        }
    }
    if ( !params.variable_name.empty() ) {
        ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
        for ( unsigned int i = 0; params.reload >= 0; i++ ) {
            ShmFw::Alloc<ShmFw::Image> shm_image( params.variable_name, shmHdl );
            if ( params.reload == 0 ) {
                shm_image.wait();
            }
            if ( i == 0 ) std::cout << shm_image.human_readable() << std::endl;
            cv::Mat img;
	    shm_image->toCvMat ( img );
            cv::imshow ( params.variable_name.c_str(), img );
            if ( params.reload == 0 ) cv::waitKey ( 10 );
            else if ( cv::waitKey ( params.reload ) >= 0 ) params.reload = -1;
        }
    }
    exit ( 0 );
}
