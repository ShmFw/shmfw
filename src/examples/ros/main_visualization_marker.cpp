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

#include <shmfw/objects/ros/header.h>
#include <shmfw/objects/ros/visualization_marker.h>
#include <shmfw/variable.h>
#include <shmfw/allocator.h>


#include <boost/program_options.hpp>
#include <boost/thread.hpp>

struct Prarmeters {
    bool clear;
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
    ( "clear,c", "clears the shared memory" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( DEFAULT_SEGMENT_NAME ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( DEFAULT_SEGMENT_SIZE ), "shared memory segment size" );

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

double rand_01() {
    return ( ( double ) rand() ) / RAND_MAX;
}

int main ( int argc, char *argv[] ) {


    Prarmeters params = readArgs ( argc, argv );
    if ( params.clear ) {
        ShmFw::Handler::removeSegment ( params.shm_memory_name );
        std::cout << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
    }
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    srand ( time ( NULL ) );

    ShmFw::Alloc<ShmFw::ros::Header> h ( "header", shmHdl );
    h().frame_id = "world";

    double r = 2, da = M_PI/64;
    int i = 0;
    
    ShmFw::Alloc<ShmFw::ros::VisualizationMarker> markerArrow ( "VisualizationMarker", shmHdl );
    ShmFw::Alloc<ShmFw::ros::VisualizationMarker> markerLineList ( "VisualizationMarkerLineList", shmHdl );
    ShmFw::Alloc<ShmFw::ros::VisualizationMarker> markerLineStrip ( "VisualizationMarkerLineStrip", shmHdl );
    for ( double alpha = 0; true ; alpha += da, i++ ) {
        double x0 = cos ( alpha ) * r, y0 = sin ( alpha ) * r;
        double x1 = cos ( alpha ) * 2*r, y1 = sin ( alpha ) * 2*r;
	markerArrow->setArrow("map",1,"markerArrow",ShmFw::RGBA::green(),ShmFw::Point ( x0,y0,0 ),ShmFw::Point ( x1,y1,0 ) );
        markerArrow.itHasChanged();
	markerLineList->setLineList("map",2,"markerLineList",ShmFw::RGBA::blue(), 0.1);
	markerLineList->addLineListElement(ShmFw::Point(x0,y0,0), ShmFw::Point(x0,y0,r));
	markerLineStrip->setLineList("map",3,"markerLineStrip",ShmFw::RGBA::red(),0.1);
	markerLineStrip->addLineStripElement(ShmFw::Point(0,0,0));
	markerLineStrip->addLineStripElement(ShmFw::Point(x0,y0,0));
	usleep(1000*100);
    }
    exit ( 0 );

}
