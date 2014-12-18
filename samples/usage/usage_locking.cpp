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

#include <shmfw/vector.h>

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#define COUT_DEFAULT std::cout << std::string("\033[0;0m")
#define COUT_GREEN std::cout << std::string("\033[32m")
#define COUT_BLUE std::cout << std::string("\033[33m")

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
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );

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

void conditionThread ( ShmFw::HandlerPtr &shmHdl, const std::string &name ) {
    int offest = 60;
    COUT_GREEN << std::setw ( offest ) << "Thread started" << std::endl;
    ShmFw::Vector<int> xy ( name, shmHdl );
    COUT_GREEN << std::setw ( offest ) << "waiting to get the look using timed_wait()" << std::endl;
    while ( xy.timed_wait ( 500 ) == false ) {
        COUT_GREEN << std::setw ( offest ) << "timed_wait(500)" << std::endl;
        std::cout << std::flush;
    }
    COUT_GREEN << std::setw ( offest ) << "timed_wait(500) finished" << std::endl;
    COUT_GREEN << std::setw ( offest ) << "lock()" << std::endl;
    xy.lock();
    COUT_GREEN << std::setw ( offest ) << "lock() finished" << std::endl;
    COUT_GREEN << std::setw ( offest ) << "push_back ( 100 )" << std::endl;
    xy->push_back ( 100 );
    COUT_GREEN << std::setw ( offest ) << "push_back ( 100 ) finished" << std::endl;
    COUT_GREEN << std::setw ( offest ) << "itHasChanged ( )" << std::endl;
    xy.itHasChanged();
    COUT_GREEN << std::setw ( offest ) << "itHasChanged ( ) finished" << std::endl;
    COUT_GREEN << std::setw ( offest ) << "unlock ( )" << std::endl;
    xy.unlock();
    COUT_GREEN << std::setw ( offest ) << "unlock ( ) finished" << std::endl;
}

int main ( int argc, char *argv[] ) {
    Prarmeters params = readArgs ( argc, argv );
    if ( params.clear ) {
        ShmFw::Handler::removeSegment ( params.shm_memory_name );
        COUT_DEFAULT << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
    }
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );

    std::string varName ( "xy" );
    ShmFw::Vector<int> xy ( varName, shmHdl );
    xy.clear();
    COUT_BLUE << "lock()" << std::endl;
    xy.lock();
    COUT_BLUE << "lock() finished" << std::endl;
    boost::thread t1 ( conditionThread, shmHdl,  varName );
    COUT_BLUE << "sleep(2)" << std::endl;
    sleep ( 2 );
    COUT_BLUE << "sleep(2) finished" << std::endl;
    COUT_BLUE << "itHasChanged()" << std::endl;
    xy.itHasChanged();
    COUT_BLUE << "itHasChanged() finished" << std::endl;
    COUT_BLUE << "unlock()" << std::endl;
    xy.unlock();
    COUT_BLUE << "unlock() finished" << std::endl;
    COUT_BLUE << "wait()" << std::endl;
    xy.wait();
    COUT_BLUE << "wait() finished" << std::endl;

    COUT_DEFAULT << "exit" << std::endl;
    exit ( 0 );

}
