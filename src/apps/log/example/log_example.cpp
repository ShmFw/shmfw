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
#include <stdarg.h>


#include "shmfw/log.h"
#include <boost/program_options.hpp>

SHMFW_INIT_LOG;

bool loop_program = true;

struct Prarmeters {
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
    ( "shm_log,l", po::value<std::string> ( &params.variable_name )->default_value ( "log" ), "shared variable name of the logger" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_LOG_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_LOG_SEGMENT_SIZE() ), "shared memory segment size" );

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

void terminate ( int param ) {
    std::cout << "Closing program!" << std::endl;
    loop_program = false;
}

int main ( int argc, char *argv[] ) {

    signal ( SIGINT, terminate );
    signal ( SIGKILL, terminate );
    Prarmeters params = readArgs ( argc, argv );
    
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Log log ( shmHdl, params.variable_name );
    std::cout << log->max_size() << std::endl;
    for ( unsigned int i = 0; loop_program; i++ ) {
        int us = rand() % 10;
        SHMFW_Info_FNC ( "debug" );
        SHMFW_Debug_FNC ( "info" );
        usleep ( us );
    }

    exit ( 0 );


}
