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


#include <opencv2/highgui/highgui.hpp>
#include <shmfw/allocator.h>
#include <shmfw/variable.h>
#include <shmfw/objects/grid_map.h>
#include <shmfw/objects/opencv.h>
#include <boost/program_options.hpp>

struct Prarmeters {
    int reload;
    std::string  cvtype;
    double grid;
    bool on_change;
    int border;
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
    ( "on_change,c", "updates only on change" )
    ( "border", po::value< int > ( &params.border )->default_value ( 20 ), "view border" )
    ( "reload,r", po::value< int > ( &params.reload )->default_value ( 100 ), "reload time in ms, 0 means reload on signal, -1 means relaod owns to load image into shm" )
    ( "cvtype,t", po::value<std::string> ( &params.cvtype ), "opencv type: CV_8UC3, CV_8U, ..." )
    ( "grid,g", po::value< double > ( &params.grid )->default_value ( 1.0 ), "grid size, -1 means no grid" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name )->default_value ( "image_grid" ), "shared variable name" );

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
    params.on_change = (vm.count ( "on_change" ) > 0); 

    return params;
}

bool loop_program = true;

void terminate ( int param ) {
    std::cout << "Program was trying to abort or terminate." << std::endl;
    loop_program = false;
}

void view_image ( const Prarmeters &params, ShmFw::HandlerPtr &shmHdl ) {
    ShmFw::Var<ShmFw::GridMap<uchar> > grid ( params.variable_name, shmHdl );
    std::cout << *grid << " --> ";
    cv::Mat img;
    if(grid->isType<double> ()){
	std::cout << " CV_64F";
	img = grid->cvMat ( CV_64F );
    } else if(grid->isType<float> ()){
	std::cout << " CV_32F";
	img = grid->cvMat ( CV_32F );
    } else if(grid->isType<int8_t> ()){
	std::cout << " CV_8S";
	img = grid->cvMat ( CV_8S );
    } else if(grid->isType<uint8_t> ()){
	std::cout << " CV_8U";
	img = grid->cvMat ( CV_8U );
    } else {
	std::cout << "no matching type!" << std::endl;
	return;
    }
    std::cout << std::endl;
    
    cv::namedWindow ( params.variable_name.c_str() );
    int key;
    do {
        cv::imshow ( params.variable_name.c_str(), img );
	if(params.on_change){
	  key = cv::waitKey ( params.reload );
	  while(grid.timed_wait(10000) == false){
	    boost::posix_time::time_duration d = ShmFw::now() - grid.timestampShm();
	    std::cout << "Last change " << d.total_milliseconds() << " ms ago!" << std::endl;
	  }
	} else {
	  key = cv::waitKey ( params.reload );
	}
    } while ( loop_program && ( key < 0 ) );
    cv::destroyWindow ( params.variable_name.c_str() );
}
int main ( int argc, char **argv ) {

    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );
    Prarmeters params = readArgs ( argc, argv );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Header *header = ( ShmFw::Header * ) shmHdl->findName ( params.variable_name );
    if ( header ) {
        view_image ( params, shmHdl );
    }


    exit ( 0 );
}
