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

#include "shmfw/variable.h"
#include <boost/program_options.hpp>

struct Prarmeters {
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    bool loop;
    Prarmeters()
    : loop(true) {
    }
};
Prarmeters params;

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "variable_name,n", po::value<std::string> ( &params.variable_name ), "shared variable name" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );

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


void terminate ( int s ) {
    printf ( "Caught signal %d\n",s );
    fflush ( stdout );
    params.loop = false;
}

int main ( int argc, char *argv[] ) {

    readArgs ( argc, argv );
    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    if ( shmHdl->findName ( params.variable_name ) == NULL ) {
        std::cerr << "No variable: " << params.variable_name;
        std::cerr << " in share segment " << params.shm_memory_name;
        std::cerr << "!" << std::endl;
        return 0;
    }
    ShmFw::Var<ShmFw::Header> header( params.variable_name, shmHdl );
    boost::posix_time::ptime t0 = boost::posix_time::microsec_clock::local_time();
    unsigned int count = 0;
    while ( params.loop ) {           // Don't echo() while we do getch
	t0 = t0 + boost::posix_time::seconds(1);
	std::vector<boost::posix_time::time_duration> cycles;
	std::vector<boost::posix_time::time_duration> cycles_times;
	while(boost::posix_time::microsec_clock::local_time() < t0){
	  if(header.timed_wait(5000)){
	    cycles.push_back(boost::posix_time::microsec_clock::local_time() - header->timestampLocal());
	    header->dataProcessed();
	  } else {
	    std::cerr << "No signal within 5sec: " << std::endl;
	  }
	}
	if(cycles.size() > 0){
	  double sum = 0;
	  for(size_t i = 0; i < cycles.size(); i++){
	    sum += cycles[i].total_microseconds() / ( double ) boost::posix_time::seconds ( 1 ).total_microseconds();
	  }
	  double cycleAVR = sum / (double) cycles.size();
	  
	  printf ( "%4i: %6.2fHz = %8.2fms :  %s", count, 1.0/cycleAVR, cycleAVR*1000, params.variable_name.c_str());
	  count++;
	}
    };
    exit ( 0 );

}



